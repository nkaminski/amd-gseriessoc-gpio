# Maintainer: Nash Kaminski <nashkaminski@comcast.net>
# Contributor: Peter Chinetti <peter@chinetti.me>

pkgname=amd-gseriessoc-gpio
pkgver=1
pkgrel=1
pkgdesc="GPIO driver for AMD G series SoC"
arch=('i686' 'x86_64')
url="http://repository.timesys.com/buildsources/a/amd-gseriessoc-gpio/"
license=('GPL2')
depends=('dkms')
conflicts=("${pkgname}")
install=${pkgname}.install
source=('amd_gseriessoc_gpio.c' 'amd_gseriessoc_gpio.h' 'Makefile')
md5sums=('e640d802a70536f9e5e6884dc32c45db'
         '61120d3f357e2ff40acd85e86f82643f'
         '42c50cce7f354560d897d11e30c172e8')
# remember to also adjust the .install files and the package deps below
_extramodules=extramodules-3.18-ARCH

build() {
  msg2 "Starting make..."
  make
}

package() {
  # Install
  msg2 "Starting install..."
  install -dm755 "$pkgdir/usr/lib/modules/$_extramodules/"
  install -m644 *.ko "$pkgdir/usr/lib/modules/$_extramodules/"
  find "$pkgdir" -name '*.ko' -exec gzip -9 {} +
}
