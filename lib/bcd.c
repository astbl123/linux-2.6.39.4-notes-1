#include <linux/bcd.h>
#include <linux/module.h>

/**
 * BCD码转换到二进制码[1]P217.
 *
 * @see Linux驱动开发入门与实战(2)
 */
unsigned bcd2bin(unsigned char val)
{
	/* 分别由低4位和高4位组成一个十进制数, 取值范围为0~99[1]P220. */
	return (val & 0x0f) + (val >> 4) * 10;
}
EXPORT_SYMBOL(bcd2bin);

/**
 *
 * 二进制码转换到BCD码[1]P217.
 *
 * @see Linux驱动开发入门与实战(2)
 */
unsigned char bin2bcd(unsigned val)
{
	return ((val / 10) << 4) + val % 10;
}
EXPORT_SYMBOL(bin2bcd);
