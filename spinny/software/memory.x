MEMORY
{
  /* Leave 16k for the UF2 bootloader */
  FLASH (rx) : ORIGIN = 0x08000000 + 8K, LENGTH = 64K - 8K
  RAM (xrw)  : ORIGIN = 0x20000000, LENGTH = 20K
}
_stack_start = ORIGIN(RAM) + LENGTH(RAM);
