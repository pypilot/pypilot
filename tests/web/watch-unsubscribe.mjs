import assert from 'node:assert/strict';
import { openControl } from './browser.mjs';
const {browser, page, errors} = await openControl();
try {
    await page.evaluate(() => {
        handlers.pypilot_values(JSON.stringify({'ap.test.gain': {profiled: true}}));
        handlers.pypilot('{"profile":"default","ap.test.gain":2}');
        URL.createObjectURL = () => 'blob:test';
        URL.revokeObjectURL = () => {};
        HTMLAnchorElement.prototype.click = () => {};
        sent.length = 0;
    });
    for (let i=0; i<2; i++) {
        await page.evaluate(() => document.querySelector('#export_profiles').click());
        await page.clock.runFor(1000);
    }
    const watches = await page.evaluate(() => sent.filter(x => x.data?.startsWith('watch=')).map(x => JSON.parse(x.data.slice(6))));
    assert.deepEqual(watches.filter(x => 'ap.test.gain' in x), [
        {'ap.test.gain': 0}, {'ap.test.gain': false},
        {'ap.test.gain': 0}, {'ap.test.gain': false}]);
    // Positive-period watches also stop, and a second stop is not sent.
    await page.evaluate(() => {sent.length=0; currentTab='Statistics'; setup_watches(); setup_watches();});
    assert.equal(await page.evaluate(() => sent.filter(x => x.data === 'watch={"ap.heading":false}').length), 1);
    assert.deepEqual(errors, []);
    console.log('PASS zero-period unsubscribe, repeated exports, positive-period unsubscribe');
} finally {await browser.close();}
