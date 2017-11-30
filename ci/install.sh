set -euxo pipefail

main() {
    # This fetches latest stable release of Cross
    local tag=$(git ls-remote --tags --refs --exit-code https://github.com/japaric/cross \
                    | cut -d/ -f3 \
                    | grep -E '^v[0.1.0-9.]+$' \
                    | $sort --version-sort \
                    | tail -n1)
    curl -LSfs https://japaric.github.io/trust/install.sh | \
        sh -s -- \
           --force \
           --git japaric/cross \
           --tag $tag

    case $TARGET in
        thumbv*-none-eabi*)
            cargo install --list | grep xargo || \
                cargo install xargo
            rustup component list | grep 'rust-src.*installed' || \
                rustup component add rust-src
            ;;
    esac
}

main
