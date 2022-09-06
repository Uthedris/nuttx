README
======

  Overview
  --------
  This document describes how to create a custom ROMFS volume that
  works with the NuttX shell (nsh).  It will be mounted at /etc and
  (by default) looks like this:

  /etc
    |
    +-- init.d
    |    |
    |    +-- rc.sysinit
    |    |
    |    +-- rcS
    |
    +-- .nshrc


  /etc/init.d/rc.sysinit
  ----------------------
  This script is run once by nsh at system startup.  It is run after 
  the board early initialization, but before any network initialization.

  The default /etc/init.d/rc.sysinit has the contents:

    mkrd -m 1 -s 512 256
    mkfatfs /dev/ram1
    mount -t vfat /dev/ram1 /tmp

  These commands create a temporary RAM disk with an empty FAT
  filesystem mounted on /tmp. For these commands to succeed, NuttX
  must be configured with the FAT filesystem enabled.


  /etc/init.d/rcS
  ---------------
  This script is run once just before the first nsh console session is
  started.  Item in this script can assume that all board and network
  initialization is complete.

  The default /etc/init.d/rcS has the contents:

    echo "NuttX startup complete."


  /etc/.nshrc
  -----------
  This script is run each time an nsh console session is started,
  provided that the CONFIG_NSH_ROMFSRC option is enabled.

  The default /etc/.nshrc has the contents:

    echo "Welcome to NuttX!"


  Updating the ROMFS File System
  ------------------------------
  The ROMFS filesystem can be modified to suit your particular needs.  You 
  can add commands to any of the existing files.  you can add completely 
  new files or directories under /etc.  You can event delete the existing
  files if you do not require them.

  If you do modify the the ROMFS, you will need to build a new nsh_romfsimg.h
  header file in the "include" directory for the board that you are building
  NuttX for. You must also insure that the CONFIG_NSH_ARCHROMFS is enabled
  (set to equal "y").

  The content of the nsh_romfsimg.h header file is generated from the
  prototype "etc" directory structure that resides in the same directory
  as this readme file.  After making desired modification to that
  directory, use the following steps from a command line session to create
  a new nsh_romfsimg.h header:

    1. Build and install the genromfs utility if not already done. This tool
       is available directly in many Linux distributions or can be built from
       source with is available at <https://sourceforge.net/projects/romfs/>.

    2. Change your current working directory to the directory that contains
       this README file.

    3. Create a new ROMFS image with the genromfs utility built in step 1:

       genromfs -d etc -f romfs_img -V NSHRomFS

    4. Convert the ROMFS image to a C header file:

        xxd -i romfs_img >nsh_romfsimg.h

       After completing this step the "romfs_img" can be safely deleted.

    5. Edit nsh_romfsimg.h to add the "const" attribute to both the romfs_img
       and romfs_img_len definitions so they will be stored in the flash rom.

    6. Move the nsh_romfsimg.h file to the include folder under the specific
       chip that you are going to build for.

    7. Insure that the CONFIG_NSH_ARCHROMFS is enabled and rebuild NuttX.
