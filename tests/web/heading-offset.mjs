import assert from 'node:assert/strict';
import { readFile } from 'node:fs/promises';
import { openControl } from './browser.mjs';

// Use the actual calibration input and JavaScript, not a copy of its limits.
const template = await readFile(new URL('../../web/templates/index.html', import.meta.url), 'utf8');
const input = template.match(/<input[^>]*id="imu_heading_offset"[^>]*>/)[0];
const {browser, page, errors} = await openControl('pypilot_control.js', input);
try {
    for (const offset of [-360, -220, -180, 0, 180, 220, 360]) {
        await page.locator('#imu_heading_offset').fill(String(offset));
        assert.equal(await page.locator('#imu_heading_offset').evaluate(el => el.checkValidity()), true);
        await page.locator('#imu_heading_offset').blur();
        const last = await page.evaluate(() => sent.filter(x => x.name === 'pypilot').at(-1));
        assert.equal(last.data, 'imu.heading_offset=' + JSON.stringify(String(offset)));
        await page.evaluate(value => handlers.pypilot(JSON.stringify({'imu.heading_offset': value})), offset);
        assert.equal(await page.locator('#imu_heading_offset').inputValue(), String(offset));
    }
    for (const offset of [-361, 361]) {
        await page.locator('#imu_heading_offset').fill(String(offset));
        assert.equal(await page.locator('#imu_heading_offset').evaluate(el => el.checkValidity()), false);
    }
    assert.deepEqual(errors, []);
    console.log('PASS full-turn heading offsets: browser validity, transmitted values, server replies, and bounds');
} finally {
    await browser.close();
}
