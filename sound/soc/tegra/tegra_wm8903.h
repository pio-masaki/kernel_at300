#include <linux/ioctl.h>  /* For IOCTL macros */

#define TEGRA_WM8903_NAME    "tegra_wm8903"
#define TEGRA_WM8903_PATH    "/dev/tegra_wm8903"

#define	TEGRA_WM8903_IOCTL_BASE	'F'

#define	TEGRA_WM8903_SET_ENABLE_FM34	_IOW(TEGRA_WM8903_IOCTL_BASE, 0, int)
#define	TEGRA_WM8903_SET_DISABLE_FM34	_IOW(TEGRA_WM8903_IOCTL_BASE, 1, int)
