import { readFile } from 'node:fs/promises';
const { chromium } = await import(process.env.PLAYWRIGHT_MODULE || 'playwright');
export async function openControl(script = 'pypilot_control.js', extraHtml = '') {
    const browser = await chromium.launch({headless: true,
        ...(process.env.CHROMIUM_EXECUTABLE ? {executablePath: process.env.CHROMIUM_EXECUTABLE} : {})});
    const page = await browser.newPage();
    const errors = [];
    page.on('pageerror', error => errors.push(error.message));
    await page.route('http://pypilot.test/', route => route.fulfill({contentType: 'text/html', body: '<html></html>'}));
    await page.goto('http://pypilot.test/');
    await page.setContent(`<table id="values"></table><div id="Control"></div><div id="gain_container"></div>
        <select id="profile"></select><button id="export_profiles">Export</button>
        <button id="import_profiles">Import</button>${extraHtml}`);
    await page.addScriptTag({content: await readFile(new URL('../../web/static/jquery.min.js', import.meta.url), 'utf8')});
    await page.evaluate(() => {
        window._ = s => s;
        window.pypilot_web_port = 8000;
        window.tinypilot = false;
        window.languages = ["en"];
        window.language = "en";
        window.handlers = {};
        window.sent = [];
        window.alerts = [];
        window.alert = s => alerts.push(s);
        window.io = {connect: () => ({
            on: (name, handler) => {handlers[name] = handler;},
            emit: (name, data) => {sent.push({name, data});}
        })};
    });
    await page.addScriptTag({content: await readFile(new URL('../../web/static/' + script, import.meta.url), 'utf8')});
    await page.waitForFunction(() => handlers.pypilot);
    await page.clock.install();
    return {browser, page, errors};
}
