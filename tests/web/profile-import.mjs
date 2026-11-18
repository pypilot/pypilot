import assert from 'node:assert/strict';
import { openControl } from './browser.mjs';
for (const scenario of ['values', 'malformed', 'timeout']) {
    const {browser, page, errors} = await openControl();
    try {
        await page.evaluate(() => handlers.pypilot_values('{}'));
        const choosing = page.waitForEvent('filechooser');
        await page.evaluate(() => document.querySelector('#import_profiles').click());
        const file = await choosing;
        const data = scenario === 'malformed' ? 'not json' : '2.5';
        await file.setFiles({name: 'profiles.conf', mimeType: 'text/plain',
            buffer: Buffer.from(`[profile="heavy"]\nap.gain=${data}\nap.flag=false\nap.pilot="basic"\n`)});
        await page.waitForFunction(() => sent.some(x => x.data === 'profile="heavy"'));
        if (scenario !== 'timeout') await page.evaluate(() => handlers.pypilot('{"profile":"heavy"}'));
        await page.clock.runFor(8000);
        const result = await page.evaluate(() => ({sent, alerts}));
        const writes = result.sent.filter(x => x.name === 'pypilot' && !x.data.startsWith('watch='));
        if (scenario === 'values') {
            assert.deepEqual(writes.map(x => x.data), ['profile="heavy"', 'ap.gain=2.5', 'ap.flag=false', 'ap.pilot="basic"', 'profile="default"']);
            assert.deepEqual(result.alerts, ['Import Complete!']);
        } else {
            assert.deepEqual(writes.map(x => x.data), ['profile="heavy"', 'profile="default"']);
            assert.equal(result.alerts.length, 1);
            assert.match(result.alerts[0], /Malformed|Failed/);
        }
        assert.deepEqual(errors, []);
        console.log('PASS import ' + scenario);
    } finally {await browser.close();}
}
