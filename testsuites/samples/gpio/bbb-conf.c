const rtems_gpio_pin_conf test[4] =
  {
    {
    pin_number: BBB_LED_USR0,
    function: DIGITAL_OUTPUT,
    pull_mode: NO_PULL_RESISTOR,
    interrupt: NULL,
    output_enabled: FALSE,
    logic_invert: FALSE,
    bsp_specific: NULL
    },
    {
    pin_number: BBB_LED_USR1,
    function: DIGITAL_OUTPUT,
    pull_mode: NO_PULL_RESISTOR,
    interrupt: NULL,
    output_enabled: FALSE,
    logic_invert: FALSE,
    bsp_specific: NULL
    },
    {
    pin_number: BBB_P8_15,
    function: DIGITAL_INPUT,
    pull_mode: PULL_UP,
    interrupt: NULL,
    output_enabled: FALSE,
    logic_invert: FALSE,
    bsp_specific: NULL
    },
    {
    pin_number: BBB_P8_16,
    function: DIGITAL_INPUT,
    pull_mode: PULL_UP,
    interrupt: NULL,
    output_enabled: FALSE,
    logic_invert: FALSE,
    bsp_specific: NULL
    }
  };