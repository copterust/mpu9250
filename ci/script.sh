set -euxo pipefail

main() {
    case $TARGET in
        thumbv*-none-eabi*)
            xargo check --target $TARGET
            ;;
        *)
            cross check --target $TARGET
            cross check --examples --target $TARGET
            ;;
    esac
}

main
