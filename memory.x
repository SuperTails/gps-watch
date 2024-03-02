MEMORY
{
  FLASH : ORIGIN = 0x08000000, LENGTH = 128K
  RAM   : ORIGIN = 0x20000000, LENGTH = 32K
}

/* Actual start of stack will be lower due to flip-link. */
/* _stack_start = ORIGIN(RAM) + LENGTH(RAM); */
