set -euxo pipefail

main() {
    cargo check --target $TARGET

    case $TARGET in
        armv*)
            cargo check --examples --target $TARGET
            ;;
        *)
            ;;
    esac
}

main
