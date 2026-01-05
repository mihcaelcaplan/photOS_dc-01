PRJ="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

$MCU_IDE -data "$MCU_WORKSPACE" -application org.eclipse.cdt.managedbuilder.core.headlessbuild -cleanBuild "photOS_dc-01/A_Partition"
$MCU_IDE -data "$MCU_WORKSPACE" -application org.eclipse.cdt.managedbuilder.core.headlessbuild -cleanBuild "photOS_dc-01/B_Partition"
$MCU_IDE -data "$MCU_WORKSPACE" -application org.eclipse.cdt.managedbuilder.core.headlessbuild -cleanBuild "photOS_dc-01/Partition_Bootloader"

BOOTLOADER_OFFSET=0
A_PARTITION_OFFSET=0x40000
B_PARTITION_OFFSET=0x3fc0000

TAG="$1"
RELEASE_FILE="photOS_release_${TAG}.bin"

dd if="$PRJ/Partition_Bootloader/photOS_dc-01_Bootloader.bin" of="$RELEASE_FILE" bs=1 seek=$((BOOTLOADER_OFFSET)) conv=notrunc
dd if="$PRJ/A_Partition/photOS_dc-01_A_Partition.bin" of="$RELEASE_FILE" bs=1 seek=$((A_PARTITION_OFFSET)) conv=notrunc
dd if="$PRJ/B_Partition/photOS_dc-01_B_Partition.bin" of="$RELEASE_FILE" bs=1 seek=$((B_PARTITION_OFFSET)) conv=notrunc

