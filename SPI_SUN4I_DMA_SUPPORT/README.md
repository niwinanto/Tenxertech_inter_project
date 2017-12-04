# DMA support for sun4i-spi device
The spi-sun4i.c file is modified to support SPI DMA communication in CHIP Pro.  
**Kernel Version : 4.4.30**  
**Patch found for Kernel Version: 4.7.0**(https://github.com/hramrach/linux-sunxi/commits/sunxi-spi-dma-merged)  
SPI communication will switch to DMA mode(**sunxi_spi_can_dma()**) when ever FIFO exceeds 63(**#define SUN4I_FIFO_DEPTH		63**). 


## Modification in the patch to support Kernel 4.4.30 
* Used dmaengine_terminate_all() instead of dmaengine_terminate_sync().  
* max_transfer_size field in new spi_master structure is not backward compatible. Since we are not using this field, commented out this line(**master->max_transfer_size = sunxi_spi_max_transfer_size;**).

