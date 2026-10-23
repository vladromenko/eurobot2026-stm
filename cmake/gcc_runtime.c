/* Newlib constructor hooks required by the existing GNU startup when using
 * -nostartfiles. Application initialization remains in SystemInit and main.
 * No semihosting, syscall redirection or changes to robot drivers.
 */
void _init(void) {}
void _fini(void) {}

/* Called if a pure virtual method is ever reached through a vtable.
 * Defined here so the linker does not pull std::terminate from libsupc++.
 */
void __cxa_pure_virtual(void) { while (1) {} }
