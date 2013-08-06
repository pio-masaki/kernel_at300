/*
 * HND SiliconBackplane PMU support.
 *
 * $Copyright Open Broadcom Corporation$
 *
 * $Id: hndpmu.h 277737 2011-08-16 17:54:59Z $
 */

#ifndef _hndpmu_h_
#define _hndpmu_h_


extern void si_pmu_otp_power(si_t *sih, osl_t *osh, bool on);
extern void si_sdiod_drive_strength_init(si_t *sih, osl_t *osh, uint32 drivestrength);

#endif /* _hndpmu_h_ */
