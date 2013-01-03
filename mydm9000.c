#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/init.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/crc32.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
//#include <linux/dm9000.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <asm/delay.h>
#include <asm/irq.h>
#include <asm/io.h>
#include <mach/gpio.h> /*定义GPIO管脚功能*/
#include <mach/regs-mem.h> /*定义S5P_SROM_BW寄存器的地址*/
#include "dm9000-big.h"
#include "dm9000.h"
MODULE_AUTHOR("AKAE");
MODULE_DESCRIPTION("Sample DM9000 driver");
MODULE_LICENSE("Dual BSD/GPL");

#define DEVICE_NAME "dm9000"

/*board_info结构体，用来保存芯片相关的一些私有信息*/
typedef struct board_info {
	void __iomem *io_addr; /*映射到Linux内存空间的地址口虚拟地址*/
	void __iomem *io_data; /*映射到Linux内存空间的数据口虚拟地址*/
	u16 irq; /*IRQ*/
	u16 tx_pkt_cnt; /*待发送数据包数量，最多只能有两个*/
	u16 queue_pkt_len; /*待发送数据包长度*/
	u8 imr_all; /*用于给DM9000_IMR赋值的一个变量*/
	struct resource *addr_res; /*地址口地址*/
	struct resource *data_res; /*数据口地址*/
	struct resource *irq_res; /*IRQ*/
	struct mutex addr_lock; /*互斥信号两*/
	spinlock_t lock; /*自旋锁*/
} board_info_t;

/*Read a byte from I/O port*/
static u8 ior(board_info_t * db, int reg)
{
	writeb(reg, db->io_addr);
	return readb(db->io_data);
}

/*Write a byte to I/O port*/
static void iow(board_info_t * db, int reg, int value)
{
	writeb(reg, db->io_addr); /*将寄存器地址写入映射进内存的i\o内存空间，通过地址找到相应的寄存器*/
	writeb(value, db->io_data); /*将数据写入寄存器*/
}

/*调用时机：当网卡有数据需要发送的时候，该函数被调用*/
/*第二个包的发送将在dm9000_tx_done中实现，这是因为当第一个数据发送完之后会产生一个中断，则会调用dm9000_tx_done对应的函数*/
static int dm9000_start_xmit(struct sk_buff *skb, struct net_device *dev)
{
	//printk("%s is calling...\n", __func__);
	unsigned long flags;
	board_info_t *db = netdev_priv(dev);
	/*获得自旋锁并禁止中断，中断状态保存在flags中*/
	spin_lock_irqsave(&db->lock, flags);
	/*将io_addr写入寄存器MWCMD（写数据到DM9000内的发送SRAM中），进行这个操作后，向io_data写入的数据会传输到dm9000内部TX_SRAM中*/
	writeb(DM9000_MWCMD, db->io_addr);
	writesw(db->io_data, skb->data, (skb->len+1) >> 1);
	/*待发送的数据包数量（tx_pkt_cnt）加1*/
	db->tx_pkt_cnt++;
	/* TX control: First packet immediately send, second packet queue */
	if (db->tx_pkt_cnt == 1) {
		iow(db, DM9000_TXPLL, skb->len);
		iow(db, DM9000_TXPLH, skb->len >> 8);
		iow(db, DM9000_TCR, TCR_TXREQ);//第一个包的发送命令。
	} else {
		/* Second packet */
		db->queue_pkt_len = skb->len;
		netif_stop_queue(dev);//第二个包了，没空间装了，要停止上层协议栈的发送队列。
	}
	/*释放自旋锁，并恢复中断之前的状态*/
	spin_unlock_irqrestore(&db->lock, flags);
	/*每个数据包写入网卡SRAM后都要释放skb*/
	dev_kfree_skb(skb);
	return NETDEV_TX_OK;
}

static void dm9000_tx_done(struct net_device *dev, board_info_t *db)
{
	//printk("%s is calling...\n", __func__);
	/*读取dm9000寄存器NSR（NetworkStatus Register）获取发送的状态，存在变量tx_status中*/
	int tx_status = ior(db, DM9000_NSR);
	/*若有数据包发送完成，则进入*/
	if (tx_status & (NSR_TX2END | NSR_TX1END)) {
		/*待发送的数据包数量（tx_pkt_cnt）减1*/
		db->tx_pkt_cnt--;
		/*已发送的数据包数量（stats.tx_packets）加1*/
		//dev->stats.tx_packets++;
		/*检查变量tx_pkt_cnt是否大于0（如果大于0，表明还有数据包要发送），则再次发送*/
		if (db->tx_pkt_cnt > 0)
		{
			/*将要发送的数据包的长度写入DM9000的两个发送数据包长度寄存器中*/
			iow(db, DM9000_TXPLL, db->queue_pkt_len);
			iow(db, DM9000_TXPLH, db->queue_pkt_len >> 8);
			/*提出发送请求，发送完成后该位自动清零*/
			iow(db, DM9000_TCR, TCR_TXREQ);
		}
		netif_wake_queue(dev);//有任何一个包发完，就表示硬件缓冲区有空间，可以打开上层协议栈发送队列了。
	}
}

/*该结构体封装了dm9000接收的数据包的头信息*/
struct dm9000_rxhdr {
	u8 RxPktReady;
	u8 RxStatus;
	__le16 RxLen;
} __attribute__((__packed__));/*告诉编译器取消结构在编译过程中的优化*/

static void dm9000_rx(struct net_device *dev)
{
	//printk("%s is calling...\n", __func__);
	board_info_t *db = netdev_priv(dev);
	struct dm9000_rxhdr rxhdr;
	struct sk_buff *skb;
	u8 rxbyte, *rdptr;
	int RxLen; /*接收数据包的长度*/
	/* Check packet ready or not */
	do {
		/* 读取寄存器MRCMDX,MRCMDX寄存器地址保存在了db->io_addr中，下面要读取MRCMDX寄存器的值只需要读取db->io_data即可*/
		ior(db, DM9000_MRCMDX);
		/*读取MRCMDX寄存器的值，rxbyte为8位，因为MRCMDX寄存器存储的值就是8位*/
		rxbyte = readb(db->io_data);
		/*DM9000_PKT_RDY定义为0x01，而rxbyte为我们读取的第一个字节，其值只能是0x00（表示还没接收）或0x01（表示已经接收）*/
		if (!(rxbyte & DM9000_PKT_RDY))
		{
			//printk("rxbyte is not DM9000_PKT_RDY!\n");
			return;
		}
		/*将MRCMD寄存器地址写入db->io_addr*/
		writeb(DM9000_MRCMD, db->io_addr);
		/*一次性从MRCMD寄存器（即RX_SRAM）中读入四个字节的内容到rxhdr变量，*/
		readsw(db->io_data, &rxhdr, (sizeof(rxhdr)+1) >> 1);
		/*获取接收数据包的长度*/
		RxLen = le16_to_cpu(rxhdr.RxLen);
		//printk("RxLen=%d\n", RxLen);
		skb = dev_alloc_skb(RxLen + 4);
		skb_reserve(skb, 2);
		//减去的4个字节是网卡硬件自动增加的4个字节的CRC校验码，不需要保留在sk_buff里。下面读的时候是读走的，但是skb->len里没计入。
		rdptr = (u8 *) skb_put(skb, RxLen - 4);
		/*读取数据放入rdptr所在位置，rdptr位置为skb_tail，所以这句话就是读取RX_SRAM内容到skb*/
		readsw(db->io_data, rdptr, (RxLen+1) >> 1);
		//dev->stats.rx_bytes += RxLen;
		/*函数eth_type_trans用于从以太网数据包中提取网络协议内容，并把它放入skb结构的相应位置*/
		skb->protocol = eth_type_trans(skb, dev);
		/*调用netif_rx将数据交给协议栈*/
		netif_rx(skb);
	} while (rxbyte & DM9000_PKT_RDY);
}

/*网卡有三种类型的中断：新报文到达中断，报文发送完成中断，出错中断*/
static irqreturn_t dm9000_interrupt(int irq, void *dev_id)
{
	//printk("%s is calling...\n", __func__);
	struct net_device *dev = dev_id;
	board_info_t *db = netdev_priv(dev);
	int int_status;
	unsigned long flags;
	u8 reg_save;
	/*获得自旋锁并禁止中断，中断状态保存在flags中*/
	//spin_lock_irqsave(&db->lock, flags);
	/*保存中断前的状态*/
	reg_save = readb(db->io_addr);
	/* 禁用DM9000中断*/
	iow(db, DM9000_IMR, IMR_PAR);
	/*中断状态寄存器，当一个中断到来时，该寄存器存放着中断类型。DM9000中断处理函数通过读取该寄存器，得到目前中断信息*/
	/*读取该中断状态寄存器之后，还需要将读取结果存放回该寄存器，也就是需要清除中断状态，否则将无法再次响应中断*/
	int_status = ior(db, DM9000_ISR);
	iow(db, DM9000_ISR, int_status);
	/*检测中断状态寄存器，如果是由于收到数据而触发的中断，显然调用dm9000_rx()把数据取走，传递给上层*/
	if (int_status & ISR_PRS)
		dm9000_rx(dev);
	/*检测中断状态寄存器，如果是由于发送完了数据而触发的中断，则调用dm9000_tx_done()函数*/
	if (int_status & ISR_PTS)
		dm9000_tx_done(dev, db);
	/*重新使能DM9000各中断功能*/
	iow(db, DM9000_IMR, db->imr_all);
	/*恢复中断前的状态*/
	writeb(reg_save, db->io_addr);
	/*自旋锁解锁*/
	//spin_unlock_irqrestore(&db->lock, flags);
	return IRQ_HANDLED;
}

/*当使用命令ifconfig eth0 up时，网卡被打开，执行这一函数,向内核注册中断，复位并初始化dm9000，检查MII接口，使能传输等*/
static int dm9000_open(struct net_device *dev)
{
	/*返回board_info_t的地址*/
	board_info_t *db = netdev_priv(dev);
	/*IRQF_TRIGGER_MASK为中断触发方式，定义在Interrupt.h中*/
	unsigned long irqflags = db->irq_res->flags & IRQF_TRIGGER_MASK;
	/*设置为共享中断*/
	irqflags |= IRQF_SHARED;
	/*申请中断并注册中断服务程序*/
	request_irq(dev->irq, dm9000_interrupt, irqflags, dev->name, dev);
	/*复位dm9000*/
	iow(db, DM9000_NCR, 1);
	do {
		udelay(100);
	} while (ior(db, DM9000_NCR) & 0x1);
	/*初始化DM9000*/
	/*DM9000的GPIO0默认为输出做POWER_DOWN功能，默认值为1，表明要使PHY层POWER_DOWN，即不启用PHY，若希望启用PHY，则驱动程序需要通过写“0”将PWER_DOWN信号清零*/
	iow(db, DM9000_GPR, 0); /* Enable PHY */
	/*使能数据包接收中断，使能数据包传输中断*/
	db->imr_all = IMR_PAR | IMR_PTM | IMR_PRM;
	iow(db, DM9000_IMR, db->imr_all);
	/*设置DM9000的RCR接收控制寄存器的第0位为1，接收使能*/
	/*忽略CRC校验错误的包、忽略长度超大的包*/
	iow(db, DM9000_RCR, RCR_DIS_LONG | RCR_DIS_CRC | RCR_RXEN);

	//打开上层协议栈发送队列
	netif_start_queue(dev);

	return 0;
}
/*驱动支持的网卡设备操作函数*/
static const struct net_device_ops dm9000_netdev_ops = {
	.ndo_open = dm9000_open,
	.ndo_start_xmit = dm9000_start_xmit,
};
/*内核加载驱动后，自动执行驱动的probe函数，进行资源的探测和申请资源*/
static int __devinit dm9000_probe(struct platform_device *pdev)
{
	struct board_info *db; /*将获得的资源信息存放在这个结构体里*/
	struct net_device *ndev; /*定义一个网络设备*/
	int iosize;
	int i, oft;
	u32 id_val;
	/*分配ndev，使用alloc_etherdev()函数分配一个网络设备的结构体，原型在include/linux/etherdevice.h */
	ndev = alloc_etherdev(sizeof(struct board_info));
	/*通过SET_NETDEV_DEV()将platform_device与net_device接洽关联起来*/
	SET_NETDEV_DEV(ndev, &pdev->dev);
	/*下面都是设置board_info结构体，将获得的资源信息存放在这个结构体里*/
	/*struct board_info *db和struct net_device *ndev都是局部变量。但是又需要board_info和net_device二者建立一一对应关系，而board_info 是一个自定义结构，通过netdev_priv(ndev)就把board_info放入net_device里，建立了联系。创建net_device 时已经为board_info 留了空间：ndev = alloc_etherdev(sizeof(struct board_info));*/
	db = netdev_priv(ndev);/*将网络设备结构与自定义设备结构建立关联，获取net_device结构的私有成员保存到struct board_info *db中*/
	spin_lock_init(&db->lock); /*初始化自旋锁*/
	mutex_init(&db->addr_lock); /*初始化互斥信号量*/
	db->addr_res = platform_get_resource(pdev, IORESOURCE_MEM, 0);        /*获取DM9000地址口的资源地址*/
	db->data_res = platform_get_resource(pdev, IORESOURCE_MEM, 1);        /*获取DM9000数据口的资源地址*/
	db->irq_res = platform_get_resource(pdev, IORESOURCE_IRQ, 0); 	      /*获取DM9000中断资源*/
	iosize = resource_size(db->addr_res);
	/*将地址口映射到linux内存空间的虚拟地址*/
	db->io_addr = ioremap(db->addr_res->start, iosize);
	iosize = resource_size(db->data_res);
	/*将数据口映射到linux内存空间的虚拟地址*/
	db->io_data = ioremap(db->data_res->start, iosize);
	/*****************************设置结构体board_info结束***************************************************/
	/*填充net_device结构体*/
	ndev->base_addr = (unsigned long)db->io_addr; /*设置网络设备地址*/
	ndev->irq = db->irq_res->start; /*设置网络设备中断资源地址*/
	/*获取DM9000的ID号，需要多次获得以免失败*/
	for (i = 0; i < 8; i++) {
		id_val = ior(db, DM9000_VIDL);
		id_val |= (u32)ior(db, DM9000_VIDH) << 8;
		id_val |= (u32)ior(db, DM9000_PIDL) << 16;
		id_val |= (u32)ior(db, DM9000_PIDH) << 24;
		if (id_val == DM9000_ID)
			break;
	}
	/*借助ether_setup()函数来部分初始化ndev。因为对以太网设备来讲，很多操作与属性是固定的，内核可以帮助完成*/
	ether_setup(ndev);
	/*驱动支持的网卡设备操作函数*/
	ndev->netdev_ops = &dm9000_netdev_ops;

	//没有EEPROM，手动赋值MAC地址：
	ndev->dev_addr[0] = 0x08;
	ndev->dev_addr[1] = 0x01;
	ndev->dev_addr[2] = 0x02;
	ndev->dev_addr[3] = 0x03;
	ndev->dev_addr[4] = 0x04;
	ndev->dev_addr[5] = 0x05;

	/*注册ndev网络设备*/
	register_netdev(ndev);

	for (i = 0, oft = DM9000_PAR; i < 6; i++, oft++)
		iow(db, oft, ndev->dev_addr[i]);

	return 0;
}

static struct platform_driver dm9000_driver = {
	.driver = {
		.name = "dm9000", /*驱动的名字*/
		.owner = THIS_MODULE,
	},
	.probe = dm9000_probe, /*重要的函数：资源探测函数*/
};

/*第一步：加载网卡驱动首先要被执行的*/
static int __init dm9000_init(void)
{
	printk(KERN_INFO "%s Ethernet Driver\n", DEVICE_NAME);
	return platform_driver_register(&dm9000_driver); /*调用函数platform_driver_register()函数注册驱动*/
}

static void __exit dm9000_cleanup(void)
{
	platform_driver_unregister(&dm9000_driver);
}

module_init(dm9000_init);
module_exit(dm9000_cleanup);
