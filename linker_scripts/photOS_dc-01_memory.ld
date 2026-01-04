/*
 * GENERATED FILE - DO NOT EDIT
 * Copyright 2008-2013 Code Red Technologies Ltd,
 * Copyright 2013-2026 NXP
 * SPDX-License-Identifier: BSD-3-Clause
 * Generated linker script file for MIMXRT1062xxxxB
 * Created from memory.ldt by FMCreateLinkMemory
 * Using Freemarker v2.3.30
 * MCUXpresso IDE v24.12 [Build 148] [2025-01-10] on Jan 3, 2026, 5:29:01 PM
 */

MEMORY
{
  /* Define each memory region */
  App_Bootloader (rx) : ORIGIN = 0x60000000, LENGTH = 0x40000 /* 256K bytes (alias Flash) */  
  A_Partition (rx) : ORIGIN = 0x60040000, LENGTH = 0x3f80000 /* 65024K bytes (alias Flash2) */  
  B_Partition (rx) : ORIGIN = 0x63fc0000, LENGTH = 0x3f80000 /* 65024K bytes (alias Flash3) */  
  SRAM_DTC (rwx) : ORIGIN = 0x20000000, LENGTH = 0x20000 /* 128K bytes (alias RAM) */  
  SRAM_ITC (rwx) : ORIGIN = 0x0, LENGTH = 0x20000 /* 128K bytes (alias RAM2) */  
  SRAM_OC2 (rwx) : ORIGIN = 0x20200000, LENGTH = 0x80000 /* 512K bytes (alias RAM3) */  
  SRAM_OC (rwx) : ORIGIN = 0x20280000, LENGTH = 0x40000 /* 256K bytes (alias RAM4) */  
  NCACHE_REGION (rwx) : ORIGIN = 0x80000000, LENGTH = 0x1000000 /* 16M bytes (alias RAM5) */  
  BOARD_SDRAM (rwx) : ORIGIN = 0x81000000, LENGTH = 0x1000000 /* 16M bytes (alias RAM6) */  
}

  /* Define a symbol for the top of each memory region */
  __base_App_Bootloader = 0x60000000  ; /* App_Bootloader */  
  __base_Flash = 0x60000000 ; /* Flash */  
  __top_App_Bootloader = 0x60000000 + 0x40000 ; /* 256K bytes */  
  __top_Flash = 0x60000000 + 0x40000 ; /* 256K bytes */  
  __base_A_Partition = 0x60040000  ; /* A_Partition */  
  __base_Flash2 = 0x60040000 ; /* Flash2 */  
  __top_A_Partition = 0x60040000 + 0x3f80000 ; /* 65024K bytes */  
  __top_Flash2 = 0x60040000 + 0x3f80000 ; /* 65024K bytes */  
  __base_B_Partition = 0x63fc0000  ; /* B_Partition */  
  __base_Flash3 = 0x63fc0000 ; /* Flash3 */  
  __top_B_Partition = 0x63fc0000 + 0x3f80000 ; /* 65024K bytes */  
  __top_Flash3 = 0x63fc0000 + 0x3f80000 ; /* 65024K bytes */  
  __base_SRAM_DTC = 0x20000000  ; /* SRAM_DTC */  
  __base_RAM = 0x20000000 ; /* RAM */  
  __top_SRAM_DTC = 0x20000000 + 0x20000 ; /* 128K bytes */  
  __top_RAM = 0x20000000 + 0x20000 ; /* 128K bytes */  
  __base_SRAM_ITC = 0x0  ; /* SRAM_ITC */  
  __base_RAM2 = 0x0 ; /* RAM2 */  
  __top_SRAM_ITC = 0x0 + 0x20000 ; /* 128K bytes */  
  __top_RAM2 = 0x0 + 0x20000 ; /* 128K bytes */  
  __base_SRAM_OC2 = 0x20200000  ; /* SRAM_OC2 */  
  __base_RAM3 = 0x20200000 ; /* RAM3 */  
  __top_SRAM_OC2 = 0x20200000 + 0x80000 ; /* 512K bytes */  
  __top_RAM3 = 0x20200000 + 0x80000 ; /* 512K bytes */  
  __base_SRAM_OC = 0x20280000  ; /* SRAM_OC */  
  __base_RAM4 = 0x20280000 ; /* RAM4 */  
  __top_SRAM_OC = 0x20280000 + 0x40000 ; /* 256K bytes */  
  __top_RAM4 = 0x20280000 + 0x40000 ; /* 256K bytes */  
  __base_NCACHE_REGION = 0x80000000  ; /* NCACHE_REGION */  
  __base_RAM5 = 0x80000000 ; /* RAM5 */  
  __top_NCACHE_REGION = 0x80000000 + 0x1000000 ; /* 16M bytes */  
  __top_RAM5 = 0x80000000 + 0x1000000 ; /* 16M bytes */  
  __base_BOARD_SDRAM = 0x81000000  ; /* BOARD_SDRAM */  
  __base_RAM6 = 0x81000000 ; /* RAM6 */  
  __top_BOARD_SDRAM = 0x81000000 + 0x1000000 ; /* 16M bytes */  
  __top_RAM6 = 0x81000000 + 0x1000000 ; /* 16M bytes */  
