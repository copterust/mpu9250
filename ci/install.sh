set -euxo pipefail

main() {
    case $TARGET in
        thumbv*-none-eabi*)
            # This fetches latest stable release of Xargo
            local tag=$(git ls-remote --tags --refs --exit-code https://github.com/japaric/xargo \
                            | cut -d/ -f3 \
                            | grep -E '^v[0.3.0-9.]+$' \
                            | sort --version-sort \
                            | tail -n1)

            curl -LSfs https://japaric.github.io/trust/install.sh | \
                sh -s -- \
                   --force \
                   --git japaric/xargo \
                   --tag $tag
            ;;
        *)
            # This fetches latest stable release of Cross
            local tag=$(git ls-remote --tags --refs --exit-code https://github.com/japaric/cross \
                            | cut -d/ -f3 \
                            | grep -E '^v[0.1.0-9.]+$' \
                            | sort --version-sort \
                            | tail -n1)

            curl -LSfs https://japaric.github.io/trust/install.sh | \
                sh -s -- \
                   --force \
                   --git japaric/cross \
                   --tag $tag
            ;;
    esac
}

main
