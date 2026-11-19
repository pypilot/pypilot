import assert from 'node:assert/strict';
import { openControl } from './browser.mjs';
const {browser, page, errors} = await openControl();
try {
    const count = () => page.evaluate(() => sent.filter(x => x.name === 'ping').length);
    await page.evaluate(() => handlers.connect());
    await page.clock.runFor(5000);
    assert.equal(await count(), 1);
    await page.evaluate(() => handlers.disconnect?.());
    await page.clock.runFor(10000);
    assert.equal(await count(), 1, 'do not queue pings while disconnected');
    await page.evaluate(() => handlers.connect());
    await page.clock.runFor(5000);
    assert.equal(await count(), 2, 'one ping after reconnect');
    // Repeated connect notifications must not accumulate more timers.
    await page.evaluate(() => {handlers.connect(); handlers.connect();});
    await page.clock.runFor(10000);
    assert.equal(await count(), 4);
    assert.deepEqual(errors, []);
    console.log('PASS initial connection, disconnect, reconnect and repeated connect');
} finally {await browser.close();}
