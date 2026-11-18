import assert from 'node:assert/strict';
import { openControl } from './browser.mjs';

for (const failedSwitch of [false, true]) {
    const {browser, page, errors} = await openControl();
    try {
        await page.evaluate(() => {
            handlers.pypilot_values(JSON.stringify({'ap.test.gain': {profiled: true}}));
            handlers.pypilot(JSON.stringify({profiles: ['default', 'heavy'], profile: 'default'}));
            window.downloads = [];
            URL.createObjectURL = blob => {downloads.push(blob); return 'blob:test';};
            URL.revokeObjectURL = () => {};
            HTMLAnchorElement.prototype.click = () => {};
            document.querySelector('#export_profiles').click();
        });
        if (!failedSwitch) {
            await page.evaluate(() => handlers.pypilot(JSON.stringify({profile: 'heavy', 'ap.test.gain': 9})));
            await page.clock.runFor(1000);
            await page.evaluate(() => handlers.pypilot(JSON.stringify({profile: 'default', 'ap.test.gain': 2})));
            await page.clock.runFor(1000);
            assert.equal(await page.evaluate(() => downloads[0].text()),
                '[profile="heavy"]\nap.test.gain=9\n[profile="default"]\nap.test.gain=2\n');
        } else {
            await page.clock.runFor(1000);
            const result = await page.evaluate(() => ({alerts, sent, downloads: downloads.length}));
            assert.equal(result.downloads, 0);
            assert.deepEqual(result.alerts, ['Failed to capture profiles for export!']);
            assert.ok(result.sent.some(x => x.data === 'profile="default"'), 'restore original profile');
        }
        assert.deepEqual(errors, []);
        console.log(`PASS export ${failedSwitch ? 'failed switch cleanup' : 'distinct profile values'}`);
    } finally { await browser.close(); }
}
