set -euxo pipefail

main() {
    case $TARGET in
        thumbv*-none-eabi*)
            xargo check --target $TARGET
            ;;
        *)
            cargo check --target $TARGET
            cargo check --examples --target $TARGET
            ;;
    esac
}

main
