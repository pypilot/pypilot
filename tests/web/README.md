# Web regression checks

Install Playwright separately (`npm install playwright`) and install Chromium (`npx playwright install chromium`). Set PLAYWRIGHT_MODULE to the absolute path to Playwright's index.mjs if it is not available through normal Node module resolution. CHROMIUM_EXECUTABLE optionally selects an installed Chromium.

Run a regression with Node, for example `node tests/web/profile-export.mjs`.

These load the real web JavaScript and bundled jQuery in Chromium. Socket.IO is replaced with recorded messages and simulated server replies; browser time is controlled to exercise timers without long waits. They do not connect to an autopilot or test physical steering equipment.
