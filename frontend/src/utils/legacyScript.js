const LEGACY_SCRIPT_ATTR = "data-reino-script";
const LEGACY_SCRIPT_VERSION = "20251122-1";
const LEGACY_SCRIPT_URL = `/static/js/script.js?v=${LEGACY_SCRIPT_VERSION}`;

export const getLegacyScript = () =>
    document.querySelector(`script[${LEGACY_SCRIPT_ATTR}="true"]`);

export const ensureLegacyScript = () => {
    let script = getLegacyScript();
    if (!script) {
        script = document.createElement("script");
        script.type = "text/javascript";
        script.setAttribute(LEGACY_SCRIPT_ATTR, "true");
        document.body.appendChild(script);
    }

    if (script.src !== window.location.origin + LEGACY_SCRIPT_URL) {
        script.src = LEGACY_SCRIPT_URL;
    }

    return script;
};

export const LEGACY_SCRIPT_CONSTANTS = {
    ATTR: LEGACY_SCRIPT_ATTR,
    VERSION: LEGACY_SCRIPT_VERSION,
    URL: LEGACY_SCRIPT_URL,
};
