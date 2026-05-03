/* Copyright (C) 2026 Sean D'Epagnier <seandepagnier@gmail.com>
 *
 * This Program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <sys/ioctl.h>

#include "spireader.h"

static int spi_open(int portn, int slave, unsigned int baud)
{
    int fd;
    uint8_t mode = SPI_MODE_0;
    uint8_t bits = 8;
    uint32_t speed = baud;

    char dev[64];
    snprintf(dev, sizeof dev, "/dev/spidev%d.%d", portn, slave);
    fd = open(dev, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "open %s ", dev);
        perror("");
        exit(1);
    }

    if (ioctl(fd, SPI_IOC_WR_MODE, &mode) < 0) {
        perror("SPI_IOC_WR_MODE");
        exit(1);
    }

    if (ioctl(fd, SPI_IOC_WR_BITS_PER_WORD, &bits) < 0) {
        perror("SPI_IOC_WR_BITS_PER_WORD");
        exit(1);
    }

    if (ioctl(fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0) {
        perror("SPI_IOC_WR_MAX_SPEED_HZ");
        exit(1);
    }

    return fd;
}

spireader::spireader(int ms, int repeatn)
    : repeat(repeatn)
{
    ts.tv_sec = 0;
    ts.tv_nsec = ms * 1e6;
}

int spireader::open(int portn, int slave, int baud)
{
    port = slave;
    spifd = spi_open(portn, slave, baud);

    return spifd;
}

spireader::~spireader()
{
    close();
}

void spireader::close()
{
    ::close(spifd);
}

static void spi_transfer(int fd,
                         const uint8_t *tx,
                         uint8_t *rx,
                         size_t len)
{
    struct spi_ioc_transfer tr;

    memset(&tr, 0, sizeof(tr));

    tr.tx_buf = (unsigned long)tx;
    tr.rx_buf = (unsigned long)rx;
    tr.len = len;
    tr.speed_hz = 0;
    tr.bits_per_word = 8;
    tr.delay_usecs = 0;
    tr.cs_change = 0;

    int ret = ioctl(fd, SPI_IOC_MESSAGE(1), &tr);
    if (ret < 0) {
        perror("SPI_IOC_MESSAGE");
        exit(1);
    }

    if ((size_t)ret != len) {
        fprintf(stderr, "short SPI transfer: %d of %zu\n", ret, len);
        exit(1);
    }
}

unsigned char spireader::blocking_read_byte()
{
    unsigned char c = 0;
    unsigned char d;
    for(int i=0; i < repeat; i++) {
        spi_transfer(spifd, &c, &d, 1);
        if(d)
            return d;
        nanosleep(&ts, 0);
    }
    spi_transfer(spifd, &c, &d, 1);
    return d;
}

unsigned char spireader::xfer_byte(unsigned char c)
{
    unsigned char d;
    spi_transfer(spifd, &c, &d, 1);
    return d;
}


std::vector<uint8_t> spireader::xfer(const std::vector<uint8_t>& tx)
{
    int len = tx.size();
    std::vector<uint8_t> rx(len);
    spi_transfer(spifd, tx.data(), rx.data(), len);
    printf("xfer %d\n",len);
    for(int i=0; i<len; i++)
        printf("%d %x %x\n", i, tx[i], rx[i]);
    return rx;
}
