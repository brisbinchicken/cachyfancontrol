# Maintainer: HearneLabs <michael@hearnelabs.local>
pkgname=cachyfancontrol
pkgver=0.1.0
pkgrel=1
pkgdesc="Native CachyOS (Arch Linux) fan control daemon and Wayland GUI reverse-engineered from FanControl"
arch=('x86_64' 'aarch64')
url="https://github.com/brisbinchicken/CachyFanControl"
license=('MIT')
depends=(
    'gcc-libs'
    'glibc'
    'lm_sensors'
    'wayland'
    'libxkbcommon'
    'libglvnd'
)
makedepends=()
optdepends=(
    'rust: system Rust toolchain if not using rustup'
    'nct6687d-dkms-git: Super I/O driver for Nuvoton NCT6687-R motherboards'
    'it87-dkms-git: Super I/O driver for ITE IT87xx motherboards'
)
install=cachyfancontrol.install
source=()
sha256sums=()

prepare() {
    export PATH="$HOME/.cargo/bin:$PATH"
    if ! command -v cargo >/dev/null 2>&1; then
        echo "ERROR: 'cargo' not found. Install via 'sudo pacman -S rust' or 'rustup'." >&2
        return 1
    fi
    cd "$startdir"
    cargo fetch --locked --target "$(rustc -vV | sed -n 's/host: //p')" || cargo fetch
}

build() {
    export PATH="$HOME/.cargo/bin:$PATH"
    export CARGO_TARGET_DIR="$srcdir/target"
    cd "$startdir"
    cargo build --frozen --release || cargo build --release
}

check() {
    export PATH="$HOME/.cargo/bin:$PATH"
    export CARGO_TARGET_DIR="$srcdir/target"
    cd "$startdir"
    cargo test --frozen --release || cargo test --release
}

package() {
    cd "$startdir"

    # Install daemon and Wayland GUI binaries
    install -Dm755 "$srcdir/target/release/cachyfancontrold" "$pkgdir/usr/bin/cachyfancontrold"
    install -Dm755 "$srcdir/target/release/cachyfancontrol-gui" "$pkgdir/usr/bin/cachyfancontrol-gui"

    # Install systemd service, sysusers, tmpfiles, and udev rules
    install -Dm644 "packaging/cachyfancontrold.service" "$pkgdir/usr/lib/systemd/system/cachyfancontrold.service"
    install -Dm644 "packaging/cachyfancontrol.sysusers" "$pkgdir/usr/lib/sysusers.d/cachyfancontrol.conf"
    install -Dm644 "packaging/cachyfancontrol.tmpfiles" "$pkgdir/usr/lib/tmpfiles.d/cachyfancontrol.conf"
    install -Dm644 "packaging/99-cachyfancontrol.rules" "$pkgdir/usr/lib/udev/rules.d/99-cachyfancontrol.rules"

    # Install Wayland desktop entry, icons, license & documentation
    install -Dm644 "packaging/cachyfancontrol.desktop" "$pkgdir/usr/share/applications/cachyfancontrol.desktop"
    install -Dm644 "assets/cachyfancontrol.svg" "$pkgdir/usr/share/icons/hicolor/scalable/apps/cachyfancontrol.svg"
    install -Dm644 "assets/logo.png" "$pkgdir/usr/share/pixmaps/cachyfancontrol.png"
    install -Dm644 "LICENSE" "$pkgdir/usr/share/licenses/$pkgname/LICENSE"
    install -Dm644 "README.md" "$pkgdir/usr/share/doc/$pkgname/README.md"
}
