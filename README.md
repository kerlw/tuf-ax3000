
注意：
=
1. **不**要用 **root** 用户 git 和编译！！！
2. 国内用户编译前最好准备好梯子

## 编译

1. 首先装好 Ubuntu 64bit，推荐  Ubuntu  18 LTS x64 /  Mint 19.1

2. 命令行输入 `sudo apt-get update` ，然后输入
`
sudo apt-get -y install build-essential asciidoc binutils bzip2 gawk gettext git libncurses5-dev libz-dev patch python3.5 python2.7 unzip zlib1g-dev lib32gcc1 libc6-dev-i386 subversion flex uglifyjs git-core gcc-multilib p7zip p7zip-full msmtp libssl-dev texinfo libglib2.0-dev xmlto qemu-utils upx libelf-dev autoconf automake libtool autopoint device-tree-compiler g++-multilib antlr3 gperf wget libncurses5:i386 libelf1:i386 lib32z1 lib32stdc++6 gtk-doc-tools intltool binutils-dev cmake lzma liblzma-dev lzma-dev uuid-dev liblzo2-dev xsltproc dos2unix libstdc++5 docbook-xsl-* sharutils autogen shtool gengetopt
`

3. 使用 `git clone https://github.com/MerlinRdev/tuf-ax3000` 命令下载好源代码，然后 `cd tuf-ax3000/release/src-rt-5.02axhnd.675x` 进入目录

4. 分别执行 `sudo mkdir /opt/toolchains/`,  `sudo ln -sf $(pwd)/bcmdrivers/broadcom/net/wl/impl61/main/src/toolchains/crosstools-aarch64-gcc-5.3-linux-4.1-glibc-2.24-binutils-2.25 /opt/toolchains/`,  `sudo ln -sf $(pwd)/bcmdrivers/broadcom/net/wl/impl61/main/src/toolchains/crosstools-aarch64-gcc-5.5-linux-4.1-glibc-2.26-binutils-2.28.1 /opt/toolchains/`,  `sudo ln -sf $(pwd)/bcmdrivers/broadcom/net/wl/impl61/main/src/toolchains/crosstools-arm-gcc-5.3-linux-4.1-glibc-2.24-binutils-2.25 /opt/toolchains/`,  `sudo ln -sf $(pwd)/bcmdrivers/broadcom/net/wl/impl61/main/src/toolchains/crosstools-arm-gcc-5.5-linux-4.1-glibc-2.26-binutils-2.28.1 /opt/toolchains/`,  `sudo ln -sf $(pwd)/bcmdrivers/broadcom/net/wl/impl61/main/src/toolchains/crosstools-gcc-5.3-linux-4.1-uclibc-1.0.12-glibc-2.24-binutils-2.25 /opt/toolchains/`, `chsh -s /bin/bash` 

5. 输入 `make tuf-ax3000` 即可开始编译你要的固件了。

6. 编译完成后输出固件路径：tuf-ax3000/release/src-rt-5.02axhnd.675x/image

## Donate

如果你觉得此项目对你有帮助，请捐助我们，以使项目能持续发展，更加完善。

### PayPal

[![Support via PayPal](https://cdn.rawgit.com/twolfson/paypal-github-button/1.0.0/dist/button.svg)](https://paypal.me/paldier/)

### Alipay 支付宝

![alipay](doc/alipay_donate.jpg)

### Wechat 微信
  
![wechat](doc/wechat_donate.jpg)


