const SECTIONS = {
    'link-doxygen':  'doxygen/index.html',
    'link-coverage': 'coverage/index.html',
    'link-tests':    'test-results/index.html',
};

async function loadMetrics() {
    try {
        const res = await fetch('./metrics.json');
        const data = await res.json();

        document.getElementById('lastUpdated').textContent = data.timestamp ? new Date(data.timestamp * 1000).toLocaleString() : '--';
    } catch {
        console.warn('metrics.json not available');
    }
}

function updateLinks() {
    for (const [id, path] of Object.entries(SECTIONS)) {
        const el = document.getElementById(id);
        if (el) el.href = `./${path}`;
    }
}

updateLinks()
loadMetrics().then(r => console.log('Metrics loaded'));