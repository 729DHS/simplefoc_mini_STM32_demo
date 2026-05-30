#
# openocd-helpers.tcl — 精简版（无线调试用）
# 原版 CDLiveWatchSetup 在 elaphureLink 无线连接下会卡住
# 此处保留 Cortex-Debug 需要的 proc 定义，但跳过自动调用
#

set USE_SWO 0

proc CDSWOConfigure { CDCPUFreqHz CDSWOFreqHz CDSWOOutput } {
    catch {tpiu init}
    set tipu_names [tpiu names]
    if { [llength $tipu_names] == 0 } {
        puts stderr "[info script]: Error: Could not find TPIU/SWO names."
    } else {
        set mytpiu [lindex $tipu_names 0]
        $mytpiu configure -protocol uart -output $CDSWOOutput -traceclk $CDCPUFreqHz -pin-freq $CDSWOFreqHz
        $mytpiu enable
    }
}

proc CDRTOSConfigure { rtos } {
    set target [target current]
    if { $target != "" } {
        $target configure -rtos $rtos
    }
}

proc CDLiveWatchSetup {} {
    puts "[info script]: CDLiveWatchSetup skipped (wireless DAP)"
}

# 注意：不在这里调用 CDLiveWatchSetup
# 原版在这行会卡住无线连接，已移除
