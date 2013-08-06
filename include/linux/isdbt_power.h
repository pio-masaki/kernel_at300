#ifndef _ISDBT_POWER_H
#define _ISDBT_POWER_H


struct isdbt_platform_data {
    unsigned int    isdb_1v8;
    unsigned int    isdb_3v3;
    unsigned int    isdbt_reset_pin;
    int (*enable_isdbt_regulator_control)(int enable);
    int (*isdbt_regulator_status)(void);

};

#endif
