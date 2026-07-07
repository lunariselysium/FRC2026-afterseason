from __future__ import annotations

import argparse

from .config import load_config
from .server import FloortrackServer


def main() -> None:
    parser = argparse.ArgumentParser(description="Run Floortrack field vision.")
    parser.add_argument("--config", default=None, help="Path to a Floortrack JSON config.")
    parser.add_argument("--demo", action="store_true", help="Run the browser UI with synthetic data.")
    parser.add_argument("--host", default=None, help="Override the configured HTTP host.")
    parser.add_argument("--port", type=int, default=None, help="Override the configured HTTP port.")
    args = parser.parse_args()

    config = load_config(args.config)
    if args.host is not None or args.port is not None:
        from dataclasses import replace

        config = replace(
            config,
            server=replace(
                config.server,
                host=args.host if args.host is not None else config.server.host,
                port=args.port if args.port is not None else config.server.port,
            ),
        )

    if args.demo:
        from .demo import DemoProvider

        provider = DemoProvider(config)
    else:
        from .pipeline import VisionPipeline

        provider = VisionPipeline(config)

    provider.start()
    server = FloortrackServer(config, provider)
    url = f"http://{config.server.host}:{config.server.port}/"
    print(f"Floortrack serving {url}")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        provider.stop()


if __name__ == "__main__":
    main()
