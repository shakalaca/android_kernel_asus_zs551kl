/*
 * Copyright (C) Fuzhou Rockchip Electronics Co.Ltd
 * Author: Tusson <dusong@rock-chips.com>
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/firmware.h>
#include <linux/delay.h>

#include "isp-fw.h"
#include "spi2apb.h"

static int spi_read_wait(struct spi_device *spi,
        const struct rkl_section *sec)
{
    int32_t value = 0;
    int try = 0;
    int ret = 0;

    do {
        ret = spi2apb_safe_r32(spi, sec->wait_addr, &value);

        if (!ret && value == sec->wait_value)
            break;

        if (try++ == sec->timeout) {
            ret = -1;
            dev_err(&spi->dev, "read 0x%x is %x != %x timeout\n",
                    sec->wait_addr, value, sec->wait_value);
            break;
        }
        mdelay(sec->wait_time);
    } while (1);

    return ret;
}

static int spi_boot_request(struct spi_device *spi,
        const struct rkl_section * sec)
{
    struct rkl_boot_request boot_req;
    int try = 0;
    int ret = 0;

    //send boot request to dsp for ddr init
    boot_req.flag = sec->flag;
    boot_req.loadAddr = sec->load_addr;
    boot_req.bootLen = sec->size;
    boot_req.status = 1;
    boot_req.cmd = 2;

    ret = spi2apb_safe_write(spi, BOOT_REQUEST_ADDR,
            (int32_t*)&boot_req, sizeof(boot_req));
    if (ret)
        return ret;

    if (sec->flag & BOOT_FLAG_READ_WAIT) {
        //waitting for dsp init ddr done
        do {
            ret = spi2apb_safe_read(spi, BOOT_REQUEST_ADDR,
                    (int32_t*)&boot_req, sizeof(boot_req));

            if (!ret && boot_req.status == 0)
                break;

            if (try++ == sec->timeout) {
                ret = -1;
                dev_err(&spi->dev, "boot_request timeout\n");
                break;
            }
            mdelay(sec->wait_time);
        } while (1);
    }

    return ret;
}


static int spi_download_section(struct spi_device *spi,
        const uint8_t *data, const struct rkl_section *sec, uint8_t * section_data, uint32_t fw_speed)
{
    int ret = 0;

    /*dev_info(&spi->dev, "offset:%x,size:%d,addr:%x,"
            "wait_time:%x,timeout:%x,crc:%x,flag:%x,type:%x",
            sec->offset, sec->size, sec->load_addr, sec->wait_time,
            sec->timeout, sec->crc_16, sec->flag, sec->type);
    */
    if (sec->size > 0) {
        if(sec->size > fw_section_size){
            pr_err("[preisp] spi_download_section sec->size(%d) > %d",sec->size ,fw_section_size);
            return -1;
        }
        memcpy(section_data, data + sec->offset, sec->size);
        spi->max_speed_hz = fw_speed;
        
        ret = spi2apb_safe_write(spi, sec->load_addr,
                (int32_t*)(section_data), sec->size);
        if (ret)
            return ret;
    }
    spi->max_speed_hz = fw_speed /2;
    if (sec->flag & BOOT_FLAG_BOOT_REQUEST) {
        ret = spi_boot_request(spi, sec);
    } else if (sec->flag & BOOT_FLAG_READ_WAIT) {
        ret = spi_read_wait(spi, sec);
    }

    return ret;
}

/**
 * spi_download_fw: - rk preisp firmware download through spi
 *
 * @spi: spi device
 * @fw_name: name of firmware file, NULL for default firmware name
 * Context: can sleep
 *
 * It returns zero on success, else a negative error code.
 **/



int spi_download_fw(struct spi_device *spi, const char *fw_name, uint32_t fw_speed, uint8_t * section_data)
{
    const struct rkl_header * head;
    const struct firmware *fw;
    int i = 0;
    int ret = 0;

    if (fw_name == NULL)
        fw_name = RKL_DEFAULT_FW_NAME;

    dev_info(&spi->dev, "before request firmware");
    ret = request_firmware(&fw, fw_name, &spi->dev);
    if (ret) {
        dev_err(&spi->dev, "request firmware %s failed!", fw_name);
        return ret;
    }

    head = (const struct rkl_header *) fw->data;

    dev_info(&spi->dev, "request firmware %s (version:%s) success!", fw_name, head->version);

    //section_data = (uint8_t *)kmalloc(fw_section_size, GFP_KERNEL | GFP_DMA);
    memset(section_data, 0, fw_section_size);
    if(section_data){
        for (i = 0; i < head->section_count; i++) {
            ret = spi_download_section(spi, fw->data, &head->sections[i], section_data, fw_speed);
            if (ret)
                break;
        }
        //kfree(section_data);
    }else{
        ret = -1;
        pr_err("%s section_data no memory!\n",__func__);
    }
    release_firmware(fw);
    return ret;
}
