import assert from 'node:assert/strict';
import { openControl } from './browser.mjs';
for (const answer of [null, 'heavy', 'new']) {
    const {browser, page, errors} = await openControl();
    try {
        await page.evaluate(answer => {
            document.body.insertAdjacentHTML('beforeend', '<button id="add_profile">Add</button>');
            handlers.pypilot_values('{}');
            handlers.pypilot('{"profiles":["default","heavy"],"profile":"default"}');
            window.prompt = () => answer;
            window.downloads = [];
            URL.createObjectURL = blob => {downloads.push(blob);return 'blob:test';};
            URL.revokeObjectURL = () => {};
            HTMLAnchorElement.prototype.click = () => {};
            sent.length=0;
            document.querySelector('#add_profile').click();
        }, answer);
        const writes = await page.evaluate(() => sent.filter(x => !x.data.startsWith('watch=')));
        assert.deepEqual(writes, answer === 'new' ? [{name:'pypilot', data:'profile="new"'}] : []);
        // Export before any server confirmation. It must still restore default.
        await page.evaluate(() => document.querySelector('#export_profiles').click());
        assert.equal(await page.evaluate(() => sent.at(-1).data), 'profile="heavy"');
        await page.evaluate(() => handlers.pypilot('{"profile":"heavy"}'));
        await page.clock.runFor(1000);
        if (answer === 'new') {
            assert.equal(await page.evaluate(() => sent.at(-1).data), 'profile="new"');
            await page.evaluate(() => handlers.pypilot('{"profile":"new"}'));
            await page.clock.runFor(1000);
        }
        assert.equal(await page.evaluate(() => sent.at(-1).data), 'profile="default"');
        assert.deepEqual(errors, []);
        console.log('PASS add profile ' + JSON.stringify(answer));
    } finally {await browser.close();}
}
