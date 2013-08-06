struct ast_tspdrv_platform_data {
    int vibrator_en_pin;
    int pwm_id;
    unsigned long pwm_period;
    unsigned long pwm_duty_cycle;
    char *rail_name;
};
