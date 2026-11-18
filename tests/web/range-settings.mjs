import assert from 'node:assert/strict';
import { openControl } from './browser.mjs';
const {browser, page, errors} = await openControl('pypilot_client.js');
try {
    await page.evaluate(() => {
        handlers.pypilot_values(JSON.stringify({
            'ap.tack.delay': {type:'RangeSetting', min:0, max:60, units:'sec', writable:true},
            'wind.filter_constant': {type:'RangeProperty', min:.01, max:1, writable:true},
            'wind.speed': {type:'SensorValue'},
        }));
        handlers.pypilot(JSON.stringify({'ap.tack.delay':12, 'wind.filter_constant':.2, 'wind.speed':10}));
    });
    assert.equal(await page.locator('#val_ap_tack_delay').count(), 1, 'RangeSetting needs an editor');
    assert.equal(await page.locator('#val_ap_tack_delay').inputValue(), '12');
    assert.equal(await page.locator('#val_ap_tack_delay').getAttribute('max'), '60');
    assert.equal(await page.locator('#val_wind_filter_constant').inputValue(), '0.2');
    assert.equal(await page.locator('#val_wind_speed').count(), 0, 'sensor remains read-only');
    await page.evaluate(() => {
        const input=document.querySelector('#val_ap_tack_delay');
        input.value='15'; input.dispatchEvent(new Event('change', {bubbles:true}));
    });
    assert.ok(await page.evaluate(() => sent.some(x => x.data === 'ap.tack.delay=15')));
    await page.evaluate(() => handlers.pypilot('{"ap.tack.delay":18}'));
    assert.equal(await page.locator('#val_ap_tack_delay').inputValue(), '18');
    assert.deepEqual(errors, []);
    console.log('PASS RangeSetting edit and updates, existing RangeProperty, read-only sensor');
} finally {await browser.close();}
