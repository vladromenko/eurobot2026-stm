/* Newlib constructor hooks required by the existing GNU startup when using
 * -nostartfiles. Application initialization remains in SystemInit and main.
 * No semihosting, syscall redirection or changes to robot drivers.
 */
void _init(void) {}
void _fini(void) {}
