import React, { useState, useEffect, useRef } from "react";
import "./Dashboard.css";
import { ensureLegacyScript } from "../utils/legacyScript";

const Dashboard = () => {
    const [status, setStatus] = useState({
        battery: 85,
        mode: "LOADING...",
        voltage: 23.8,
        main_ac: "0 KWH",
    });
    const logoWhiteRefs = useRef([]);
    const animationsInitialized = useRef(false);

    useEffect(() => {
        // Set body class and opacity
        document.body.style.opacity = "1";
        document.body.className = "dashboard-page";

        // Ensure legacy animation script is loaded (cache bust safe)
        ensureLegacyScript();

        // Wait for all libraries to be loaded
        const checkLibraries = setInterval(() => {
            if (window.bodymovin && window.gsap && window.Swiper) {
                clearInterval(checkLibraries);

                // Initialize Lottie animations
                setTimeout(() => {
                    if (!animationsInitialized.current) {
                        animationsInitialized.current = true;

                        logoWhiteRefs.current.forEach((brandLogo) => {
                            if (brandLogo && brandLogo.children.length === 0) {
                                brandLogo.innerHTML = "";
                                window.bodymovin.loadAnimation({
                                    container: brandLogo,
                                    path: "/static/json/logo-white.json",
                                    renderer: "canvas",
                                    loop: false,
                                    autoplay: true,
                                    name: "Brand Animation White",
                                });
                            }
                        });

                        if (window.initializeReinoPage) {
                            window.initializeReinoPage();
                        }
                    }
                }, 100);
            }
        }, 50);

        const libraryTimeout = setTimeout(() => {
            clearInterval(checkLibraries);
        }, 5000);

        // Fetch status
        const fetchStatus = async () => {
            try {
                const response = await fetch("/api/status");
                const data = await response.json();
                setStatus(data);
            } catch (error) {
                console.error("Error fetching status:", error);
            }
        };

        fetchStatus();
        const interval = setInterval(fetchStatus, 1000);

        return () => {
            clearInterval(checkLibraries);
            clearInterval(interval);
            clearTimeout(libraryTimeout);

            if (window.gsap) {
                window.gsap.killTweensOf("*");
            }
            if (window.ScrollTrigger) {
                window.ScrollTrigger.getAll().forEach((trigger) =>
                    trigger.kill()
                );
            }

            document.body.className = "";
            animationsInitialized.current = false;
        };
    }, []);

    // Calculate battery stroke dashoffset
    const batteryCircumference = 534;
    const batteryOffset =
        batteryCircumference - (batteryCircumference * status.battery) / 100;

    return (
        <>
            <input id="home_url" type="hidden" value={window.location.origin} />
            <input
                id="current_url"
                type="hidden"
                value={window.location.href}
            />
            <div className="viewport">
                <div className="main">
                    <div className="layer layer-front">
                        {/* Header */}
                        <header>
                            <div className="container">
                                <div className="brand">
                                    <a
                                        className="hoverable-sm brand-logo logo-white"
                                        href="/"
                                        ref={(el) => {
                                            if (
                                                el &&
                                                !logoWhiteRefs.current.includes(
                                                    el
                                                )
                                            ) {
                                                logoWhiteRefs.current.push(el);
                                            }
                                        }}
                                    ></a>
                                </div>
                                <div className="navigation">
                                    <div className="navigation-header">
                                        <div className="navigation-brand">
                                            <a
                                                className="anchor brand-logo logo-white"
                                                href="/"
                                                ref={(el) => {
                                                    if (
                                                        el &&
                                                        !logoWhiteRefs.current.includes(
                                                            el
                                                        )
                                                    ) {
                                                        logoWhiteRefs.current.push(
                                                            el
                                                        );
                                                    }
                                                }}
                                            ></a>
                                        </div>
                                        <button
                                            className="btn-menu-trigger"
                                            type="button"
                                        >
                                            <i className="fa-regular fa-fw fa-x"></i>
                                        </button>
                                    </div>
                                    <ul className="navigation-menu">
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm"
                                                href="/"
                                            >
                                                <span>Studio</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm active"
                                                href="/dashboard"
                                            >
                                                <span>Dashboard</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm"
                                                href="/works"
                                            >
                                                <span>Works</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm"
                                                href="/contact"
                                            >
                                                <span>Contact</span>
                                            </a>
                                        </li>
                                    </ul>
                                </div>
                                <button
                                    className="btn-menu-trigger"
                                    type="button"
                                >
                                    <i className="fa-regular fa-fw fa-bars"></i>
                                </button>
                            </div>
                        </header>

                        {/* Main Content */}
                        <main className="dashboard-main">
                            {/* ENERGY FLOW Section (Left) */}
                            <section className="card energy-flow">
                                <div className="card-header">
                                    <h2>SLAM MAP</h2>
                                </div>
                                <div className="energy-content">
                                    <div className="slam-map-container">
                                        <img
                                            id="slamMap"
                                            className="slam-map-img"
                                            src="/slam/map"
                                            alt="SLAM Map"
                                        />
                                    </div>
                                </div>
                                <div className="battery-info">
                                    <div className="battery-time">
                                        <span className="time-value">12</span>
                                        <span className="time-label">HR</span>
                                        <span className="battery-label">
                                            BATTERY LIFE
                                        </span>
                                    </div>
                                    <div className="load-indicator">
                                        <span className="load-label">
                                            MEDIUM LOAD
                                        </span>
                                        <div className="load-bar"></div>
                                    </div>
                                </div>
                            </section>

                            {/* Robot Card Section (Right Top) */}
                            <section className="card robot-card">
                                <div className="robot-header">
                                    <h2 className="robot-title">TURTLEBOT-3</h2>
                                </div>
                                <div className="camera-label">
                                    TURTLEBOT3 CAMERA 320×240 @ 30FPS
                                </div>
                                <div className="camera-view">
                                    <img
                                        id="cameraStream"
                                        src="/camera/stream"
                                        alt="TurtleBot3 Camera"
                                    />
                                </div>
                                <div className="power-control">
                                    <span className="power-label">POWER</span>
                                    <button className="power-btn active">
                                        <i className="fas fa-power-off"></i>
                                    </button>
                                    <span className="power-status">
                                        ON{" "}
                                        <span className="status-off">OFF</span>
                                    </span>
                                </div>
                            </section>

                            {/* Details Section */}
                            <section className="card details-card">
                                <h3 className="card-title">DETAILS</h3>
                                <div className="detail-grid">
                                    <div className="detail-item">
                                        <span className="detail-label">
                                            Mode
                                        </span>
                                        <span className="detail-value">
                                            <i className="fas fa-bolt"></i>
                                            {status.mode}
                                        </span>
                                    </div>
                                    <div className="detail-item">
                                        <span className="detail-label">
                                            Time
                                        </span>
                                        <span className="detail-value">
                                            <i className="far fa-clock"></i>
                                            16H 11M
                                        </span>
                                    </div>
                                    <div className="detail-item">
                                        <span className="detail-label">
                                            Frequency
                                        </span>
                                        <span className="detail-value">
                                            <i className="fas fa-circle-notch"></i>
                                            50 HZ
                                        </span>
                                    </div>
                                    <div className="detail-item">
                                        <span className="detail-label">
                                            Temp
                                        </span>
                                        <span className="detail-value">
                                            <i className="fas fa-temperature-high"></i>
                                            30°C
                                        </span>
                                    </div>
                                    <div className="detail-item">
                                        <span className="detail-label">
                                            Ampere
                                        </span>
                                        <span className="detail-value">
                                            <i className="fas fa-charging-station"></i>
                                            600A
                                        </span>
                                    </div>
                                    <div className="detail-item">
                                        <span className="detail-label">
                                            Current
                                        </span>
                                        <span className="detail-value">
                                            <i className="fas fa-cog"></i>
                                            10A
                                        </span>
                                    </div>
                                </div>
                            </section>

                            {/* Cable Section */}
                            <section className="card cable-card">
                                <div className="cable-visual">
                                    <div className="cable-connector">
                                        <div className="connector-part"></div>
                                        <div className="cable-wire"></div>
                                        <div className="connector-part"></div>
                                    </div>
                                    <div className="time-indicator">
                                        <span className="time-remain">
                                            0 15
                                        </span>
                                        <span className="time-unit">1w</span>
                                    </div>
                                </div>
                                <div className="cable-control">
                                    <span className="cable-mode">
                                        FLASH MODE
                                    </span>
                                    <span className="cable-name">CABEL</span>
                                    <div className="toggle-switch">
                                        <span className="toggle-label">ON</span>
                                        <div className="toggle-slider"></div>
                                    </div>
                                </div>
                            </section>

                            {/* Voltage Display Section */}
                            <section className="card voltage-card">
                                <div className="voltage-display">
                                    <div className="voltage-meter">
                                        <div className="meter-bars">
                                            <div className="bar active"></div>
                                            <div className="bar active"></div>
                                            <div className="bar active"></div>
                                            <div className="bar"></div>
                                            <div className="bar"></div>
                                        </div>
                                        <div className="voltage-value">
                                            {status.voltage}
                                        </div>
                                        <div className="voltage-label">
                                            VOLT
                                            <br />
                                            DISPLAY
                                        </div>
                                    </div>
                                </div>
                                <div className="voltage-info">
                                    <div className="info-item">
                                        <span className="info-label">AC</span>
                                        <span className="info-value">
                                            23.8V
                                        </span>
                                    </div>
                                    <div className="info-divider">/</div>
                                    <div className="info-item">
                                        <span className="info-label">DC</span>
                                        <span className="info-value">14V</span>
                                    </div>
                                    <button className="details-btn">
                                        <span>Details</span>
                                        <i className="fas fa-chevron-right"></i>
                                    </button>
                                </div>
                            </section>

                            {/* Charging Mode Section */}
                            <section className="card charging-card">
                                <h3 className="card-title">CHARGING MODE</h3>
                                <div className="charging-display">
                                    <div className="charging-circle">
                                        <svg
                                            className="progress-ring"
                                            width="200"
                                            height="200"
                                        >
                                            <circle
                                                className="progress-ring-bg"
                                                cx="100"
                                                cy="100"
                                                r="85"
                                            ></circle>
                                            <circle
                                                className="progress-ring-fill"
                                                cx="100"
                                                cy="100"
                                                r="85"
                                                style={{
                                                    strokeDasharray: 534,
                                                    strokeDashoffset:
                                                        batteryOffset,
                                                }}
                                            ></circle>
                                        </svg>
                                        <div className="charging-percent">
                                            <i className="fas fa-battery-three-quarters"></i>
                                            <span className="percent-value">
                                                {status.battery}%
                                            </span>
                                        </div>
                                    </div>
                                    <div className="charging-range">
                                        <span className="range-min">0%</span>
                                        <span className="range-max">100%</span>
                                    </div>
                                </div>
                                <div className="charging-info">
                                    <div className="info-row">
                                        <span className="info-label">
                                            Power
                                        </span>
                                        <span className="info-value">
                                            5A / 220V
                                        </span>
                                    </div>
                                    <div className="info-row">
                                        <span className="info-label">
                                            Input
                                        </span>
                                        <span className="info-value">
                                            200 KWH
                                        </span>
                                    </div>
                                </div>
                            </section>
                        </main>
                    </div>
                    <div className="layer layer-back bg-dark"></div>
                </div>
            </div>
        </>
    );
};

export default Dashboard;
