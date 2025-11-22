// Dashboard JavaScript
document.addEventListener("DOMContentLoaded", function () {
    initEnergyFlow();
    initPowerButton();
    initPortToggles();
    updateTime();
    setInterval(updateTime, 1000);
});

// Energy Flow Canvas Animation
function initEnergyFlow() {
    const canvas = document.getElementById("energyCanvas");
    if (!canvas) return;

    const ctx = canvas.getContext("2d");
    canvas.width = canvas.offsetWidth;
    canvas.height = canvas.offsetHeight;

    let particles = [];
    const particleCount = 50;

    // Create particles
    for (let i = 0; i < particleCount; i++) {
        particles.push({
            x: Math.random() * canvas.width,
            y: Math.random() * canvas.height,
            vx: (Math.random() - 0.5) * 2,
            vy: (Math.random() - 0.5) * 2,
            size: Math.random() * 2 + 1,
        });
    }

    function animate() {
        ctx.fillStyle = "rgba(26, 29, 35, 0.1)";
        ctx.fillRect(0, 0, canvas.width, canvas.height);

        // Draw flow lines
        ctx.strokeStyle = "rgba(255, 107, 53, 0.3)";
        ctx.lineWidth = 2;

        // Main flow curve
        ctx.beginPath();
        ctx.moveTo(100, canvas.height / 2);
        ctx.bezierCurveTo(
            canvas.width / 3,
            canvas.height / 4,
            (canvas.width * 2) / 3,
            (canvas.height * 3) / 4,
            canvas.width - 100,
            canvas.height / 2
        );
        ctx.stroke();

        // Secondary flow curve
        ctx.beginPath();
        ctx.moveTo(100, canvas.height / 2 + 50);
        ctx.bezierCurveTo(
            canvas.width / 3,
            (canvas.height * 3) / 4,
            (canvas.width * 2) / 3,
            canvas.height / 4,
            canvas.width - 100,
            canvas.height / 2 + 50
        );
        ctx.stroke();

        // Update and draw particles
        particles.forEach((p) => {
            p.x += p.vx;
            p.y += p.vy;

            if (p.x < 0 || p.x > canvas.width) p.vx *= -1;
            if (p.y < 0 || p.y > canvas.height) p.vy *= -1;

            ctx.fillStyle = "rgba(255, 107, 53, 0.6)";
            ctx.beginPath();
            ctx.arc(p.x, p.y, p.size, 0, Math.PI * 2);
            ctx.fill();
        });

        requestAnimationFrame(animate);
    }

    animate();
}

// Power Button Toggle
function initPowerButton() {
    const powerBtn = document.querySelector(".power-btn");
    const powerStatus = document.querySelector(".power-status");
    let isOn = true;

    if (powerBtn) {
        powerBtn.addEventListener("click", function () {
            isOn = !isOn;

            if (isOn) {
                powerStatus.innerHTML =
                    'ON <span class="status-off">OFF</span>';
                powerBtn.style.color = "var(--text-primary)";
            } else {
                powerStatus.innerHTML =
                    '<span class="status-off">ON</span> OFF';
                powerBtn.style.color = "var(--accent-orange)";
            }
        });
    }
}

// Port Toggles
function initPortToggles() {
    const toggles = document.querySelectorAll(".port-toggle");

    toggles.forEach((toggle) => {
        toggle.addEventListener("click", function () {
            const portItem = this.closest(".port-item");
            const isActive = this.classList.contains("on");

            if (isActive) {
                this.classList.remove("on");
                portItem.classList.remove("active");
                portItem.querySelector(".port-value").textContent = "0 KWH";
            } else {
                this.classList.add("on");
                portItem.classList.add("active");
                const randomValue = Math.floor(Math.random() * 30) + 5;
                portItem.querySelector(".port-value").textContent =
                    randomValue + " KWH";
            }
        });
    });
}

// Update Time Display
function updateTime() {
    const timeElements = document.querySelectorAll(".detail-value");
    timeElements.forEach((el) => {
        if (el.textContent.includes("H") && el.textContent.includes("M")) {
            const now = new Date();
            const hours = now.getHours();
            const minutes = now.getMinutes();
            el.innerHTML = `<i class="far fa-clock"></i> ${hours}H ${minutes}M`;
        }
    });
}

// Cable Flash Mode Animation
const cableWire = document.querySelector(".cable-wire");
if (cableWire) {
    setInterval(() => {
        cableWire.style.animation = "pulse 0.5s ease";
        setTimeout(() => {
            cableWire.style.animation = "";
        }, 500);
    }, 3000);
}

// Add pulse animation
const style = document.createElement("style");
style.textContent = `
    @keyframes pulse {
        0%, 100% { opacity: 1; }
        50% { opacity: 0.5; }
    }
`;
document.head.appendChild(style);

// Real-time Battery Simulation
function simulateBatteryUsage() {
    const batteryElement = document.querySelector(".percent-value");
    const progressRing = document.querySelector(".progress-ring-fill");

    if (batteryElement && progressRing) {
        setInterval(() => {
            let currentBattery = parseInt(batteryElement.textContent);

            // Simulate battery drain (0.1% per second)
            currentBattery = Math.max(0, currentBattery - 0.1);

            batteryElement.textContent = Math.floor(currentBattery) + "%";

            // Update progress ring
            const circumference = 534;
            const offset =
                circumference - (circumference * currentBattery) / 100;
            progressRing.style.strokeDashoffset = offset;

            // Change color based on battery level
            if (currentBattery < 20) {
                progressRing.style.stroke = "#f44336";
            } else if (currentBattery < 50) {
                progressRing.style.stroke = "#ff9800";
            } else {
                progressRing.style.stroke = "var(--accent-orange)";
            }
        }, 1000);
    }
}

// Optional: Enable battery simulation
// simulateBatteryUsage();
