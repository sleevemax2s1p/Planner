-----BEGIN PGP SIGNED MESSAGE-----
Hash: SHA512

Format: 3.0 (quilt)
Source: git
Binary: git, git-man, git-doc, git-cvs, git-svn, git-mediawiki, git-email, git-daemon-run, git-daemon-sysvinit, git-gui, gitk, git-el, gitweb, git-all
Architecture: any all
Version: 1:2.17.1-1ubuntu0.8
Maintainer: Ubuntu Developers <ubuntu-devel-discuss@lists.ubuntu.com>
Uploaders: Jonathan Nieder <jrnieder@gmail.com>, Anders Kaseorg <andersk@mit.edu>
Homepage: https://git-scm.com/
Standards-Version: 4.1.2.0
Vcs-Browser: http://repo.or.cz/w/git/debian.git/
Vcs-Git: https://repo.or.cz/r/git/debian.git/
Build-Depends: libz-dev, gettext, libpcre3-dev, libcurl4-gnutls-dev, libexpat1-dev, subversion, libsvn-perl, libyaml-perl, tcl, python, libhttp-date-perl | libtime-modules-perl, libcgi-pm-perl, liberror-perl, libmailtools-perl, cvs, cvsps, libdbd-sqlite3-perl, unzip, libio-pty-perl, debhelper (>= 9), dh-exec (>= 0.7), dh-apache2, dpkg-dev (>= 1.16.2~)
Build-Depends-Indep: asciidoc (>= 8.6.10), xmlto, docbook-xsl
Package-List:
 git deb vcs optional arch=any
 git-all deb vcs optional arch=all
 git-cvs deb vcs optional arch=all
 git-daemon-run deb vcs optional arch=all
 git-daemon-sysvinit deb vcs optional arch=all
 git-doc deb doc optional arch=all
 git-el deb vcs optional arch=all
 git-email deb vcs optional arch=all
 git-gui deb vcs optional arch=all
 git-man deb doc optional arch=all
 git-mediawiki deb vcs optional arch=all
 git-svn deb vcs optional arch=all
 gitk deb vcs optional arch=all
 gitweb deb vcs optional arch=all
Checksums-Sha1:
 cdc8ba409643f6fe19b1bf0653d60f3aac0485f3 5015484 git_2.17.1.orig.tar.xz
 edb7e220137f9209c4c6848edf6e766d4af177a3 616652 git_2.17.1-1ubuntu0.8.debian.tar.xz
Checksums-Sha256:
 79136e7aa83abae4d8a25c8111f113d3c5a63aeb5fd93cc72c26d49c6d5ba65e 5015484 git_2.17.1.orig.tar.xz
 28e4a7a0661d88b46c109fb3bc397d92aa0112eeb06a3c5940a25742ec22557e 616652 git_2.17.1-1ubuntu0.8.debian.tar.xz
Files:
 5179245515c637357b4a134e8d4e9a6f 5015484 git_2.17.1.orig.tar.xz
 49cd207c8b555599eb0acfcc2fd58a57 616652 git_2.17.1-1ubuntu0.8.debian.tar.xz
Original-Maintainer: Gerrit Pape <pape@smarden.org>

-----BEGIN PGP SIGNATURE-----

iQIzBAEBCgAdFiEEUMSg3c8x5FLOsZtRZWnYVadEvpMFAmBBDW8ACgkQZWnYVadE
vpMxNg//X5Epzl/KuWeFVJ46b3hAKUyayBBBSxgKa8n0oQDIlT+yY4dY8yVW4CHV
bLbEp5LSEpTA1MxaArY8ldknDKkTM0eOINeDMhKzaw5bO7PSiZvT8JbLositXwXQ
bcHrL+OMrN08CmO/gPmKDnrsBpaun/u0CrgUnxVScr0bk8Y8NxUvFvn3s8nzDhXA
PGiV9cuyd9ewg4nzoTcvkj4LaNKBFUKy716GYmsNXljFDnpECD3pvNeFKNVjHy6r
VSC/ZPfvSQ/frYB/+UFnDYvLcZojFlZ9uuYKEfaE3qAYTc8XbzOKZeD3INnKrCCG
XfkgsfOFYddh/nrdEVizlwODWjbF1cXs95y5Av+VQASbRYuNiXNj6nKXHDxkEY4G
30vnhC0eRuz+F6Unbk4KIYGNF6bYrraXWLIuWRioD2FAInx6RHRfWfvOAvwvJbCx
8NVmY55jkybfs4oTgg48oamFdPQvzPUWuvv8jkB0SKiS0K75KiW2MzCDPDl3EvbO
GlrEw+7PF+77+enyufWsekT3M5dKEn/taAq5SUoiKyR88GtlBJBy59jTrTQ15jJh
LOOj+Gp73K9dsP+liT/kPnOpjLbcBzG88PAyY+Z9ItVnx5qguutzMiMQUbso8BJg
NPHbNqk8RCMWDzNz9wcrY8uPAMDKc/XPjl4pFQm/ku60GTFYP24=
=bOtV
-----END PGP SIGNATURE-----
