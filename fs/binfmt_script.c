/*
 *  linux/fs/binfmt_script.c
 *
 *  Copyright (C) 1996  Martin von Löwis
 *  original #!-checking implemented by tytso.
 */

#include <linux/module.h>
#include <linux/string.h>
#include <linux/stat.h>
#include <linux/binfmts.h>
#include <linux/init.h>
#include <linux/file.h>
#include <linux/err.h>
#include <linux/fs.h>

/*
 * Check if this handler is suitable to load the interpreter identified
 * by first BINPRM_BUF_SIZE bytes in bprm->buf following "#!".
 *
 * Returns:
 *     0: success; the new executable is ready in bprm->mm.
 *   -ve: interpreter not found, or other binfmts failed to find a
 *        suitable binary.
 */
static int load_script(struct linux_binprm *bprm,struct pt_regs *regs)
{
	const char *i_arg, *i_name;
	char *cp;
	struct file *file;
	char bprm_buf_copy[BINPRM_BUF_SIZE];
	const char *bprm_old_interp_name;
	int retval;

	if ((bprm->buf[0] != '#') || (bprm->buf[1] != '!') ||
	    (bprm->recursion_depth > BINPRM_MAX_RECURSION))
		return -ENOEXEC;
	/*
	 * This section does the #! interpretation.
	 * Sorta complicated, but hopefully it will work.  -TYT
	 */

	/*
	 * Keep bprm unchanged until we known that this is a script
	 * to be handled by this loader. Copy bprm->buf for sure,
	 * otherwise returning -ENOEXEC will make other handlers see
	 * modified data.
	 */
	memcpy(bprm_buf_copy, bprm->buf, BINPRM_BUF_SIZE);

	/* Locate and truncate end of string. */
	bprm_buf_copy[BINPRM_BUF_SIZE - 1] = '\0';
	cp = strchr(bprm_buf_copy, '\n');
	if (cp == NULL)
		cp = bprm_buf_copy + BINPRM_BUF_SIZE - 1;
	*cp = '\0';
	/* Truncate trailing white-space. */
	while (cp > bprm_buf_copy) {
		cp--;
		if ((*cp == ' ') || (*cp == '\t'))
			*cp = '\0';
		else
			break;
	}
	/* Skip leading white-space. */
	for (cp = bprm_buf_copy + 2; (*cp == ' ') || (*cp == '\t'); cp++)
		/* nothing */ ;

	/*
	 * No interpreter name found? No problem to let other handlers
	 * retry, we did not change anything.
	 */
	if (*cp == '\0') 
		return -ENOEXEC;

	i_name = cp;
	i_arg = NULL;
	/* Find start of first argument. */
	for ( ; *cp && (*cp != ' ') && (*cp != '\t'); cp++)
		/* nothing */ ;
	/* Truncate and skip leading white-space. */
	while ((*cp == ' ') || (*cp == '\t'))
		*cp++ = '\0';
	if (*cp)
		i_arg = cp;

	/*
	 * So this is our point-of-no-return: modification of bprm
	 * will be irreversible, so if we fail to setup execution
	 * using the new interpreter name (i_name), we have to make
	 * sure that no other handler tries again.
	 */

	/*
	 * OK, we've parsed out the interpreter name and
	 * (optional) argument.
	 * Splice in (1) the interpreter's name for argv[0]
	 *           (2) (optional) argument to interpreter
	 *           (3) filename of shell script (replace argv[0])
	 *
	 * This is done in reverse order, because of how the
	 * user environment and arguments are stored.
	 */

	/*
	 * Ugly: we store pointer to local stack frame in bprm,
	 * so make sure to clean this up before returning.
	 */
	bprm_old_interp_name = bprm->interp;
	bprm->interp = i_name;

	retval = remove_arg_zero(bprm);
	if (retval)
		goto out;

	/*
	 * copy_strings_kernel is ok here, even when racy: since no
	 * user can be attached to new mm, there is nobody to race
	 * with and call is safe for now. The return code of
	 * copy_strings_kernel cannot be -ENOEXEC in any case,
	 * so no special checks needed.
	 */
	retval = copy_strings_kernel(1, &bprm_old_interp_name, bprm);
	if (retval < 0)
		goto out;
	bprm->argc++;
	if (i_arg) {
		retval = copy_strings_kernel(1, &i_arg, bprm);
		if (retval < 0)
			goto out;
		bprm->argc++;
	}
	retval = copy_strings_kernel(1, &bprm->interp, bprm);
	if (retval)
		goto out;
	bprm->argc++;

	/*
	 * OK, now restart the process with the interpreter's dentry.
	 * Release old file first.
	 */
	allow_write_access(bprm->file);
	fput(bprm->file);
	bprm->file = NULL;
	file = open_exec(bprm->interp);
	if (IS_ERR(file)) {
		retval = PTR_ERR(file);
		goto out;
	}
	bprm->file = file;
	/* Caveat: This also updates the credentials of the next exec. */
	retval = prepare_binprm(bprm);
	if (retval < 0)
		goto out;
	bprm->recursion_depth++;
	retval = search_binary_handler(bprm, regs);

out:
	/*
	 * Make sure we do not return local stack frame data. If
	 * it would be needed after returning, we would have needed
	 * to allocate memory or use a copy from new bprm->mm anyway.
	 */
	bprm->interp = bprm_old_interp_name;
	/*
	 * Since bprm is already modified, we cannot continue if the the
	 * handlers for starting the new interpreter have failed.
	 * Make sure that we do not return -ENOEXEC, as that would
	 * allow searching for handlers to continue.
	 */
	if (retval == -ENOEXEC)
		retval = -EINVAL;

	return retval;
}

static struct linux_binfmt script_format = {
	.module		= THIS_MODULE,
	.load_binary	= load_script,
};

static int __init init_script_binfmt(void)
{
	return register_binfmt(&script_format);
}

static void __exit exit_script_binfmt(void)
{
	unregister_binfmt(&script_format);
}

core_initcall(init_script_binfmt);
module_exit(exit_script_binfmt);
MODULE_LICENSE("GPL");
