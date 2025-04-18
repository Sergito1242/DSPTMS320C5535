/******************************************************************************/
/* LNK.CMD - COMMAND FILE FOR LINKING C PROGRAMS                              */
/*                                                                            */
/* Usage:                                                                     */
/*  cl55 <src files> -z -o<out file> -m<map file> lnk.cmd -l<RTS library>     */
/*                                                                            */
/* Description: THIS FILE IS A SAMPLE command file that can be used for       */
/*              linking programs built with the C Compiler.  Use it as a      */
/*              guideline; you  may want to change the allocation scheme      */
/*              according to the size of your program and the memory layout   */
/*              of your target system.                                        */
/*                                                                            */
/* Note: You must specify the directory in which <RTS library> is located.    */
/*       Either add a "-i<directory>" line to this file file, or use the      */
/*       system environment variable C55X_C_DIR to specify a search path for  */
/*       the libraries.                                                       */
/*                                                                            */
/******************************************************************************/

-stack    0x2000      /* Primary stack size   */
-sysstack 0x1000      /* Secondary stack size */
-heap     0x2000      /* Heap area size       */

-c                    /* Use C linking conventions: auto-init vars at runtime */
-u _Reset             /* Force load of reset interrupt handler                */

/* SPECIFY THE SYSTEM MEMORY MAP */

MEMORY
{
 PAGE 0:  /* ---- Unified Program/Data Address Space ---- */

  RAM  (RWIX) : origin = 0x000100, length = 0x01ff00  /* 128KB page of RAM   */
  ROM  (RIX)  : origin = 0x020100, length = 0x01ff00  /* 128KB page of ROM   */
  VECS (RIX)  : origin = 0xffff00, length = 0x000100  /* 256-byte int vector */

 PAGE 2:  /* -------- 64K-word I/O Address Space -------- */

  IOPORT (RWI) : origin = 0x000000, length = 0x020000
}
 
/* SPECIFY THE SECTIONS ALLOCATION INTO MEMORY */

SECTIONS
{
   .text     > ROM    PAGE 0  /* Code                       */

   /* These sections must be on same physical memory page   */
   /* when small memory model is used                       */
   .data     > RAM    PAGE 0  /* Initialized vars           */
   .bss      > RAM    PAGE 0  /* Global & static vars       */
   .const    > RAM    PAGE 0  /* Constant data              */
   .sysmem   > RAM    PAGE 0  /* Dynamic memory (malloc)    */
   .stack    > RAM    PAGE 0  /* Primary system stack       */
   .sysstack > RAM    PAGE 0  /* Secondary system stack     */
   .cio      > RAM    PAGE 0  /* C I/O buffers              */
   .args     > RAM    PAGE 0  /* Arguments to main()        */

   /* These sections may be on any physical memory page     */
   /* when small memory model is used                       */
   .switch   > RAM    PAGE 0  /* Switch statement tables    */
   .cinit    > RAM    PAGE 0  /* Auto-initialization tables */
   .pinit    > RAM    PAGE 0  /* Initialization fn tables   */

    vectors  > VECS   PAGE 0  /* Interrupt vectors          */

   .ioport   > IOPORT PAGE 2  /* Global & static IO vars    */
}
