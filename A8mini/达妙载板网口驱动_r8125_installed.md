### 达妙载板eth⽹⼝驱动安装
> **[Realtek r8125 DKMS 的git仓库链接](https://github.com/awesometic/realtek-r8125-dkms.git)**  \
> URL:https://github.com/awesometic/realtek-r8125-dkms.git \
> GitHub - awesometic/realtek-r8125-dkms: A DKMS package for easy use of Realtek r8125 driver, which supports 2.5 GbE. \
> *标签：GitHub*

### 载板⽹⼝描述

|   |   |
|---|---|
|eth0|eth1|
|非免驱千兆网口|默认免驱千兆网口：配置驱动后，原来的eth0会⾃动更名为eth1|
|   |   |

- 达妙载板主要有两个⽹⼝“eth0”和“eth1
    - 在没有配置⽹卡驱动的初期，你只能看到⼀个eth0，因为另⼀个⽹⼝没有开放
    - 在配置⽹卡驱动后原本的eth0会⾃动更名为eth1，随后新增⼀个eth0的⽹⼝


### ⽹⼝的配置⽅法：

**⽹卡的配置⼤致有三种，这⾥我们只推荐1、2两种，因为他的仓库⾥的软件包只能适配于X86平台，因此不作描述**

1. 软件源的包管理⽅法配置（推荐）

- 添加软件源

```bash 
# 添加驱动的软件源
sudo add-apt-repository ppa:awesometic/ppa
```

- 使⽤apt⼯具安装功能包

```bash
# 使用包管理器安装驱动
sudo apt install realtek-r8125-dkms
```

- 完成上述⼯作后需要重启设备
```bash
reboot
```

- 若重启后发现驱动没有正常生效，请执行以下操作并尝试方法2的操作
```bash
# 卸载软件包同时删除相关配置文件
sudo apt remove --purge realtek-r8125-dkms

# 移除 PPA 软件源
sudo add-apt-repository --remove ppa:awesometic/ppa

# 更新 apt 缓存
sudo apt update

# 清理无用的依赖包(可选)
sudo apt autoremove
```

2. 驱动仓库⾃动化脚本配置（编译源码的方式安装）

- 拉取代码仓库
```bash
git clone https://github.com/awesometic/realtek-r8125-dkms.git
```

- 使⽤⾃动安装脚本

```bash
# 在realtek-r8125-dkms/下完成操作
cd realtek-r8125-dkms/

#给脚本可执行权限‘+x’（推荐）、‘666’都可，如果您是生产环境请切勿随意使用‘777’等级权限，‘777’的一般成功率最高， 
sudo chmod -R 777 autorun.sh

# 使用sudo 命令执行挨冻安装脚本
sudo bash ./autorun.sh
```

- 完成上述⼯作后需要重启设备
```bash
reboot
```

> **如果您在使用中发现问题或者好的注意，欢迎您在仓库议题留下您的宝贵意见，或在微信群、淘宝账户联系我们**

