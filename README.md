Microtik CCR Fork
===========

This is a fork of the linux kernel at d23a9821d3972ae373357e933c8af8216d72e374 (Linux 4.10.1)
Original source can be located at kernel.org:
https://cdn.kernel.org/pub/linux/kernel/v4.x/linux-4.10.1.tar.gz

Most of the modifications related to networking on the Mikrotik CCR devices is in
drivers/net/ethernet/tile/tilegx.c

The diff between my working repo and the original source is also included.
I have also included a config should build a working kernel.

Linux kernel
============

This file was moved to Documentation/admin-guide/README.rst

Please notice that there are several guides for kernel developers and users.
These guides can be rendered in a number of formats, like HTML and PDF.

In order to build the documentation, use ``make htmldocs`` or
``make pdfdocs``.

There are various text files in the Documentation/ subdirectory,
several of them using the Restructured Text markup notation.
See Documentation/00-INDEX for a list of what is contained in each file.

Please read the Documentation/process/changes.rst file, as it contains the
requirements for building and running the kernel, and information about
the problems which may result by upgrading your kernel.
