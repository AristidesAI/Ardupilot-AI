#!/usr/bin/env python3
"""
ArduPilot Drone AI - Main Entry Point

Autonomous AI agent for ArduPilot multicopter/drone operations.
"""

import argparse
import asyncio
import signal
import sys
from pathlib import Path

import structlog

from ai.agent import DroneAgent
from config import DroneConfig, load_config

# Configure logging
structlog.configure(
    processors=[
        structlog.stdlib.filter_by_level,
        structlog.stdlib.add_logger_name,
        structlog.stdlib.add_log_level,
        structlog.stdlib.PositionalArgumentsFormatter(),
        structlog.processors.TimeStamper(fmt="iso"),
        structlog.processors.StackInfoRenderer(),
        structlog.processors.format_exc_info,
        structlog.dev.ConsoleRenderer(),
    ],
    wrapper_class=structlog.stdlib.BoundLogger,
    context_class=dict,
    logger_factory=structlog.stdlib.LoggerFactory(),
    cache_logger_on_first_use=True,
)

logger = structlog.get_logger()


async def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(
        description="ArduPilot Drone AI Agent",
    )
    parser.add_argument(
        "--connection", "-c",
        default="udp:127.0.0.1:14550",
        help="MAVLink connection string",
    )
    parser.add_argument(
        "--config",
        type=Path,
        help="Configuration file path",
    )
    parser.add_argument(
        "--sitl",
        action="store_true",
        help="Connect to SITL (localhost:14550)",
    )
    parser.add_argument(
        "-v", "--verbose",
        action="store_true",
        help="Verbose logging",
    )

    args = parser.parse_args()

    # Load configuration
    config = load_config(args.config) if args.config else DroneConfig()

    # Override connection if SITL
    connection = args.connection
    if args.sitl:
        connection = "udp:127.0.0.1:14550"

    logger.info(
        "Starting ArduPilot Drone AI",
        connection=connection,
    )

    # Create agent
    agent = DroneAgent(config=config, connection_string=connection)

    # Setup shutdown handler
    shutdown_event = asyncio.Event()

    def signal_handler():
        logger.info("Shutdown signal received")
        shutdown_event.set()

    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, signal_handler)

    # Start agent
    if not await agent.start():
        logger.error("Failed to start agent")
        return 1

    logger.info("Agent running. Press Ctrl+C to stop.")

    # Main loop
    try:
        while not shutdown_event.is_set():
            # Print status periodically
            status = agent.get_status()
            if agent.is_connected:
                telem = status.get("telemetry", {})
                pos = telem.get("position", {})
                bat = status.get("battery", {})

                logger.info(
                    "Status",
                    state=status["mission"]["state"],
                    alt=f"{pos.get('alt_rel', 0):.1f}m",
                    soc=f"{bat.get('soc_percent', 0):.0f}%",
                )

            await asyncio.sleep(5)

    except asyncio.CancelledError:
        pass

    # Stop agent
    await agent.stop()
    logger.info("Agent stopped")

    return 0


if __name__ == "__main__":
    sys.exit(asyncio.run(main()))
