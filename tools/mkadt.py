#!/usr/bin/env python3
"""Build the minimal Apple Device Tree that the SANTONI xnu needs to boot.

Only what the early boot path actually reads (see NOTES.md); every MMIO
address comes from the santoni (Redmi 4X, MSM8937) live FDT.
Usage:  mkadt.py OUT.adt [--dump] [--qemu] [--native]
"""
import hashlib, os, struct, sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import adt  # noqa: E402

# Kernel-managed RAM: strictly above Qualcomm XPU carve-outs (ending at 0x92280000)
PHYS_BASE = 0x98000000
MEM_SIZE = 0x28000000  # 640 MiB safe budget (fits within Gigabyte 2 L1 entry for 4K bootstrap pmap)
# /arm-io children are relative to this; it must not be 0 (serial_init bails)
SOC_BASE = 0x01000000

# interrupt-controller@b000000 (santoni MSM8937 QGIC2)
GICD, GICD_SIZE = 0x0b000000, 0x1000
GICC, GICC_SIZE = 0x0b002000, 0x1000
# Continuous splash simplefb left lit by Qualcomm ABL (720x1280 24-bit BGR)
FB, FB_SIZE = 0x90001000, 720 * 1280 * 3
SWEET_ECID = 0x53414e544f4e4901  # 'SANTONI' + 1, synthetic
FB_W, FB_H, FB_STRIDE = 720, 1280, 720 * 3
TIMEBASE = 19200000
VTIMER_INTID = 20
# QEMU virt (-M virt,gic-version=2 -cpu cortex-a53): GICD/GICC and the generic timer's CNTFRQ
QEMU_GICD, QEMU_GICC, QEMU_TIMEBASE = 0x08000000, 0x08010000, 62500000
QEMU_VTIMER_INTID = 27

BOOT_MANIFEST_HASH = hashlib.sha384(b'santoni boot manifest').digest()

# The iOS userland is the iPhone X (iPhone10,3 / D22AP) IPSW. MobileGestalt
# derives product type, device class, home-button-less UI, artwork etc. from
# the root identity and /product of the real D22AP device tree. That tree is
# an Apple artifact (kept out of git): extract it from the IPSW with
#   tools/remotezip.py URL DeviceTree.d22ap.im4p DIR
#   ipsw img4 im4p extract DIR/Firmware/all_flash/DeviceTree.d22ap.im4p -o d22ap.adt
D22AP_ADT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '../research/ipsw-x/d22ap.adt')


def d22ap_product(path=D22AP_ADT):
    """/product properties of the real D22AP tree, without the syscfg
    placeholders iBoot would fill in (flag bit set) and the node bookkeeping."""
    if not os.path.exists(path):
        print(f'warning: {path} missing, no /product node', file=sys.stderr)
        return None
    root, _ = adt.parse(open(path, 'rb').read())
    for c in root.children:
        if dict((k, v) for k, f, v in c.props).get('name') == b'product\0':
            return [(k, v) for k, f, v in c.props if not f and k not in ('name', 'AAPL,phandle')]
    raise SystemExit(f'{path}: no /product node')

u32 = lambda *v: struct.pack('<%dI' % len(v), *v)
u64 = lambda *v: struct.pack('<%dQ' % len(v), *v)
cstr = lambda s: s.encode() + b'\0'


def node(name, props=(), children=()):
    n = adt.Node()
    n.props = [('name', 0, cstr(name))] + [(k, 0, v) for k, v in props]
    n.children = list(children)
    return n


def build(seed=None, qemu=False, native=False):
    """qemu=True: same tree, but with QEMU virt's GIC addresses and CNTFRQ (tools/qemu-sweet.py)."""
    phys_base, mem_size = PHYS_BASE, MEM_SIZE
    gicd, gicc, timebase = (QEMU_GICD, QEMU_GICC, QEMU_TIMEBASE) if qemu else (GICD, GICC, TIMEBASE)
    seed = seed if seed is not None else os.urandom(64)
    phandle = iter(range(1, 1000))
    ph = lambda: ('AAPL,phandle', u32(next(phandle)))

    chosen = node('chosen', [
        ph(),
        ('dram-base', u64(phys_base)),
        ('dram-size', u64(mem_size)),
        ('random-seed', seed),
        ('debug-enabled', u32(1)),
        ('firmware-version', cstr('santoni-shim')),
        ('system-firmware-version', cstr('santoni-shim')),
        # iBoot's SHA-384 of the boot manifest; /sbin/mount -P 1 exits unless
        # it exists (bind mounts /private/preboot/<hex>/usr/standalone/firmware,
        # tolerating a missing directory). Fixed stand-in value.
        ('boot-manifest-hash', BOOT_MANIFEST_HASH),
        # libimg4 (cryptexd) aborts "failed to get crypto algorithm" without
        # it; real A11+ device trees say sha2-384.
        ('crypto-hash-method', cstr('sha2-384')),
        # MobileGestalt's screen class: display-scale (u32, default 0) comes
        # from /chosen, the pixel size from IOMobileFramebuffer.
        # Santoni 720x1280 @2x = 360x640 points.
        ('display-scale', u32(2)),
        # XNU derives IOPlatformUUID (gethostuuid) from these on arm; without
        # them CFPreferences cannot use kCFPreferencesCurrentHost (no ByHost
        # files), so e.g. DataMigration never records LastSystemVersion and
        # migrates on every boot. Synthetic ECID (not the phone's); chip-id
        # of the A11 that the iPhone X userland expects.
        ('unique-chip-id', struct.pack('<Q', SWEET_ECID)),
        ('chip-id', u32(0x8015)),
        # Security status as on a dev-fused, undemoted board (tuple 0,0,1,1):
        # MobileGestalt reads these from /chosen, and mobileactivationd's
        # DeviceType then takes its native "Hactivation" branch (state
        # Activated without an activation record; no SEP, no baseband here).
        ('certificate-production-status', u32(0)),
        ('effective-production-status-ap', u32(0)),
        ('certificate-security-mode', u32(1)),
        ('effective-security-mode-sep', u32(1)),
    ], [node('memory-map', [ph()]),
        # Local boot policy (kern_csr.c): SIP relaxed like `csrutil disable`
        # (lp-sip0: untrusted kexts, unrestricted FS, task_for_pid, kernel
        # debugger, dtrace, nvram), lp-sip1: unauthenticated (unsealed) root.
        # The iOS shared cache is in no trust cache, so dyld can only map it
        # with CSR_ALLOW_UNRESTRICTED_FS (vm_unix.c shared region checks).
        node('asmb', [ph(), ('lp-sip0', u64(0x6f)), ('lp-sip1', u32(1))])])

    # Qualcomm MSM8917 (riva) Quad-core Cortex-A53
    # LK boots on Cluster 1 Core 0 (MPIDR 0x100).
    # XNU and SweetCPU require the boot CPU to be logical CPU 0.
    # List the 4 cores in Cluster 1 (0x100-0x103) as cpu0-cpu3.
    cpu_phys_ids = [0x100, 0x101, 0x102, 0x103]
    cpu_nodes = []
    for i, phys_id in enumerate(cpu_phys_ids):
        cluster = 0
        core = phys_id & 3
        clk = 1401600000
        # By default, Core 0 (0x100) is marked running because LK boots on MPIDR 0x100.
        # Bare-metal shim loader dynamically updates state based on live MPIDR_EL1.
        state = 'running' if phys_id == 0x100 else 'stopped'
        cn = node(f'cpu{i}', [
            ph(),
            ('device_type', cstr('cpu')),
            ('compatible', cstr('arm,cortex-a53') + cstr('qcom,kryo470') + cstr('ARM,v8')),
            ('reg', u32(phys_id)),
            ('cpu-id', u32(i)),
            ('cluster-id', u32(cluster)),
            ('cluster-type', u32(0)),
            ('die-cluster-id', u32(cluster)),
            ('cluster-core-id', u32(core)),
            ('l2-cache-id', u32(cluster)),
            ('l3-cache-id', u32(0)),
            ('state', cstr(state)),
            ('timebase-frequency', u32(timebase)),
            ('clock-frequency', u32(clk)),
            ('bus-frequency', u32(100000000)),
            ('memory-frequency', u32(933000000)),
            ('peripheral-frequency', u32(timebase)),
            ('fixed-frequency', u32(timebase)),
        ])
        cpu_nodes.append(cn)
    cpus = node('cpus', [ph(), ('#address-cells', u32(1)),
                         ('#size-cells', u32(0))], cpu_nodes)

    gic = node('gic', [
        ph(),
        ('compatible', cstr('qcom,msm-qgic2') + cstr('arm,gic-400')),
        ('interrupt-controller', b''),
        ('#interrupt-cells', u32(3)),
        ('virtual-timer-intid', u32(QEMU_VTIMER_INTID if qemu else VTIMER_INTID)),
        ('reg', u64(gicd - SOC_BASE, GICD_SIZE, gicc - SOC_BASE, GICC_SIZE)),
    ])
    arm_io = node('arm-io', [
        ph(),
        ('device_type', cstr('sweet-io')),
        ('compatible', cstr('sweet-io') + cstr('arm-io')),
        ('#address-cells', u32(2)),
        ('#size-cells', u32(2)),
        ('ranges', u64(0, SOC_BASE, 0x100000000 - SOC_BASE)),
    ], [gic,
        # No hardware: watchdogd checks IODeviceTree:/arm-io/wdt for no-pmu and
        # otherwise crashes trying to set the PMU boot stage (customer builds
        # ignore the -wdt_no_pmu boot-arg).
        node('wdt', [ph(), ('no-pmu', u32(1))]),
        # Main display: IOMobileFramebuffer.framework counts /arm-io children
        # with this device_type and reads the canvas size from the node
        # (the presented iPhone X size, see SweetFramebuffer.cpp);
        # the kernel's AppleCLCD matches the compatible string.
        # launchd starts wifid only when the device tree has a "wlan" node
        # (com.apple.wifid _LimitLoadToDeviceTree = wlan, marconi-wifi)
        node('wlan', [
            ph(),
            ('device_type', cstr('wlan')),
            ('compatible', cstr('qcom,wcn3680')),
        ]),
        node('disp0', [
            ph(),
            ('device_type', cstr('display-subsystem')),
            ('compatible', cstr('sweet,display')),
            ('canvas-width', u32(720 if native else 1125)),
            ('canvas-height', u32(1280 if native else 2436)),
        ])])

    fbcon = node('sweet-fbcon', [
        ph(),
        ('reg', u64(FB, FB_SIZE)),
        ('width', u32(FB_W)),
        ('height', u32(FB_H)),
        ('stride', u32(FB_STRIDE)),
        ('depth', u32(24)),
    ])

    # Internal eMMC 5.1 controller (SDCC1) for bsd/dev/santoni_sdhci.c. At the
    # root, like /sweet-ufs, so reg[] is an absolute physical address and not
    # translated by /arm-io's "ranges". The 0x1000 window starting at the
    # Qualcomm core block also contains the standard SDHCI host controller
    # registers at 0x07824900 (+0x900, 0x500 bytes); hc-reg mirrors the
    # msm8937.dtsi sdhc_1 split for documentation.
    sdhci = node('santoni-sdhci', [
        ph(),
        ('compatible', cstr('qcom,sdhci-msm-v4') + cstr('generic-sdhci')),
        ('device_type', cstr('sdhci')),
        ('reg', u64(0x07824000, 0x1000)),
        ('hc-reg', u64(0x07824900, 0x500)),
        ('bus-width', u32(8)),
        ('non-removable', b''),
    ])

    # pmap_compute_max_asids requires the node even when no overrides are set.
    # An absent pmap-max-asids property selects XNU's own MAX_ASIDS default.
    # No effaceable storage: keybagd then reads /private/var/keybags/systembag.kb
    # as a plain plist instead of decrypting it with the effaceable key.
    defaults = node('defaults', [ph(), ('no-effaceable-storage', u32(1))])

    # MobileGestalt HomeButtonType = IODeviceTree:/buttons home-button-type
    # (D22AP: 2, no Home button; absent = 0 = Home-button device).
    buttons = node('buttons', [
        ph(),
        ('button-names', cstr('volup') + cstr('voldown') + cstr('hold')),
        ('home-button-type', u32(2)),
        ('opposed-power-vol-buttons', u32(1)),
    ])

    product_props = d22ap_product()
    # Our framebuffer is sRGB 8-bit. With the iPhone X's 'P3' MobileGestalt says
    # HasExtendedColorDisplay and WebKit renders page tiles into 10-bit surfaces
    # ('w30r', 'b3a8') that this kernel does not provide: Safari pages stay blank.
    if product_props:
        product_props = [(k, cstr('sRGB') if k == 'artwork-display-gamut' else v) for k, v in product_props]
    product = [node('product', [ph()] + product_props)] if product_props else []

    # Identity of the iPhone X whose userland runs here: compatible[0] is
    # hw.model/hw.target (D22AP), model is hw.machine/hw.product
    # (iPhone10,3). 'xiaomi,santoni' and qcom,msm8937 match our target silicon.
    return node('device-tree', [
        ph(),
        ('compatible', cstr('D22AP') + cstr('iPhone10,3') + cstr('xiaomi,riva') + cstr('xiaomi,santoni') + cstr('xiaomi,sweet') + cstr('qcom,msm8917') + cstr('qcom,msm8937') + cstr('AppleARM')),
        ('model', cstr('iPhone10,3')),
        ('target-type', cstr('D22')),
        ('target-sub-type', cstr('D22AP')),
        ('manufacturer', cstr('Apple Inc.')),
        ('#address-cells', u32(2)),
        ('#size-cells', u32(2)),
    ], [chosen, defaults, cpus, arm_io, fbcon, buttons, sdhci] + product)


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: mkadt.py OUT.adt [--dump] [--qemu] [--native]", file=sys.stderr)
        sys.exit(1)
    root = build(qemu='--qemu' in sys.argv, native='--native' in sys.argv)
    blob = adt.serialize(root)
    out_path = sys.argv[1]
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    open(out_path, 'wb').write(blob)
    if '--dump' in sys.argv:
        adt.dump(adt.parse(blob)[0])
    print(f'{out_path}: {len(blob)} bytes', file=sys.stderr)
