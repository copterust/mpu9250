set -euxo pipefail

main() {
    cargo check --target $TARGET

    case $TARGET in
        armv*)
            cat >> Cargo.toml <<'EOF'

[dev-dependencies]
linux-embedded-hal = "0.2.0"
EOF

            cargo check --examples --target $TARGET
            ;;
        *)
            ;;
    esac
}

main
