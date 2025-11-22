import React, { useEffect, useRef } from "react";
import { ensureLegacyScript } from "../utils/legacyScript";

const Works = () => {
    const logoBlackRefs = useRef([]);
    const animationsInitialized = useRef(false);
    const videoObserverRef = useRef(null);

    useEffect(() => {
        // Set body opacity to 1
        document.body.style.opacity = "1";

        // Ensure legacy animation script is ready with cache busting
        ensureLegacyScript();

        // Wait for all libraries to be loaded
        const checkLibraries = setInterval(() => {
            if (window.bodymovin && window.gsap && window.Swiper) {
                clearInterval(checkLibraries);

                // Initialize Lottie animations
                setTimeout(() => {
                    if (!animationsInitialized.current) {
                        animationsInitialized.current = true;

                        // Load black logos
                        logoBlackRefs.current.forEach((brandLogo) => {
                            if (brandLogo && brandLogo.children.length === 0) {
                                brandLogo.innerHTML = "";
                                window.bodymovin.loadAnimation({
                                    container: brandLogo,
                                    path: "/static/json/logo.json",
                                    renderer: "canvas",
                                    loop: false,
                                    autoplay: true,
                                    name: "Brand Animation Black",
                                });
                            }
                        });

                        if (window.initializeReinoPage) {
                            window.initializeReinoPage();
                        }

                        // Initialize video observers
                        const videos = document.querySelectorAll("video");
                        if (
                            "IntersectionObserver" in window &&
                            videos.length > 0
                        ) {
                            videoObserverRef.current = new IntersectionObserver(
                                function (entries) {
                                    entries.forEach(function (entry) {
                                        if (entry.isIntersecting) {
                                            entry.target.play();
                                        } else {
                                            entry.target.pause();
                                        }
                                    });
                                }
                            );

                            videos.forEach(function (video) {
                                videoObserverRef.current.observe(video);
                            });
                        }
                    }
                }, 100);
            }
        }, 50);

        const timeout = setTimeout(() => {
            clearInterval(checkLibraries);
        }, 5000);

        return () => {
            clearInterval(checkLibraries);
            clearTimeout(timeout);

            if (videoObserverRef.current) {
                videoObserverRef.current.disconnect();
            }

            if (window.gsap) {
                window.gsap.killTweensOf("*");
            }
            if (window.ScrollTrigger) {
                window.ScrollTrigger.getAll().forEach((trigger) =>
                    trigger.kill()
                );
            }

            animationsInitialized.current = false;
        };
    }, []);

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
                    <div
                        className="layer layer-front"
                        style={{ backgroundColor: "var(--shades03)" }}
                    >
                        <header>
                            <div className="container">
                                <div className="brand">
                                    <a
                                        className="hoverable-sm brand-logo logo-black"
                                        href="/"
                                        ref={(el) => {
                                            if (
                                                el &&
                                                !logoBlackRefs.current.includes(
                                                    el
                                                )
                                            ) {
                                                logoBlackRefs.current.push(el);
                                            }
                                        }}
                                    ></a>
                                </div>
                                <div className="navigation">
                                    <div className="navigation-header">
                                        <div className="navigation-brand">
                                            <a
                                                className="anchor brand-logo logo-black"
                                                href="/"
                                                ref={(el) => {
                                                    if (
                                                        el &&
                                                        !logoBlackRefs.current.includes(
                                                            el
                                                        )
                                                    ) {
                                                        logoBlackRefs.current.push(
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
                                                className="menu-item anchor hoverable-sm"
                                                href="/dashboard"
                                            >
                                                <span>Dashboard</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm active"
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

                        <section className="section-banner works-page">
                            <div className="container">
                                <div className="banner-content">
                                    <div className="banner-title">
                                        <h1 className="title hoverable">
                                            We embark on
                                            <br />
                                            creative journeys.
                                        </h1>
                                    </div>
                                </div>
                            </div>
                        </section>

                        <section className="section-works">
                            <div className="container">
                                <div className="works">
                                    <div className="work-item">
                                        <a
                                            className="work-item-inner anchor"
                                            href="/works/gen"
                                        >
                                            <div className="work-item-serial hoverable">
                                                01
                                            </div>
                                            <h2 className="work-item-title hoverable">
                                                GEN
                                            </h2>
                                            <div className="work-item-media">
                                                <div className="image image-hover">
                                                    <img
                                                        alt="GEN"
                                                        loading="lazy"
                                                        src="/static/images/Gen-01-scaled.jpg.webp"
                                                    />
                                                </div>
                                            </div>
                                            <div className="work-item-description hoverable-md">
                                                <p>
                                                    Building a Cohesive Identity
                                                    <br />
                                                    for Global Engineers
                                                </p>
                                            </div>
                                        </a>
                                    </div>

                                    <div className="work-item">
                                        <a
                                            className="work-item-inner anchor"
                                            href="/works/berrics"
                                        >
                                            <div className="work-item-serial hoverable">
                                                02
                                            </div>
                                            <h2 className="work-item-title hoverable">
                                                Berrics
                                            </h2>
                                            <div className="work-item-media">
                                                <div className="image image-hover">
                                                    <img
                                                        alt="Berrics"
                                                        loading="lazy"
                                                        src="/static/images/Berrics-1-scaled.jpg.webp"
                                                    />
                                                </div>
                                            </div>
                                            <div className="work-item-description hoverable-md">
                                                <p>
                                                    A Revitalized Brand for a
                                                    Connected
                                                    <br />
                                                    Skateboarding Community
                                                </p>
                                            </div>
                                        </a>
                                    </div>

                                    <div className="work-item">
                                        <a
                                            className="work-item-inner anchor"
                                            href="/works/cariuma"
                                        >
                                            <div className="work-item-serial hoverable">
                                                03
                                            </div>
                                            <h2 className="work-item-title hoverable">
                                                Cariuma
                                            </h2>
                                            <div className="work-item-media">
                                                <div className="image image-hover">
                                                    <img
                                                        alt="Cariuma"
                                                        loading="lazy"
                                                        src="/static/images/Cariuma-1-1-scaled.jpg.webp"
                                                    />
                                                </div>
                                            </div>
                                            <div className="work-item-description hoverable-md">
                                                <p>
                                                    A Blend of Sustainability,
                                                    Urban
                                                    <br />
                                                    Style, and Brazilian
                                                    Identity
                                                </p>
                                            </div>
                                        </a>
                                    </div>

                                    <div className="work-item">
                                        <a
                                            className="work-item-inner anchor"
                                            href="/works/zuso"
                                        >
                                            <div className="work-item-serial hoverable">
                                                04
                                            </div>
                                            <h2 className="work-item-title hoverable">
                                                Zuso
                                            </h2>
                                            <div className="work-item-media">
                                                <div className="video non-hoverable">
                                                    <video
                                                        autoPlay
                                                        loop
                                                        muted
                                                        playsInline
                                                    >
                                                        <source
                                                            src="/static/videos/Zuso-Video-3.mp4"
                                                            type="video/mp4"
                                                        />
                                                    </video>
                                                </div>
                                            </div>
                                            <div className="work-item-description hoverable-md">
                                                <p>
                                                    Reinventing
                                                    <br />
                                                    Culinary Experience
                                                </p>
                                            </div>
                                        </a>
                                    </div>

                                    <div className="work-item">
                                        <a
                                            className="work-item-inner anchor"
                                            href="/works/superela"
                                        >
                                            <div className="work-item-serial hoverable">
                                                05
                                            </div>
                                            <h2 className="work-item-title hoverable">
                                                Superela
                                            </h2>
                                            <div className="work-item-media">
                                                <div className="image image-hover">
                                                    <img
                                                        alt="Superela"
                                                        loading="lazy"
                                                        src="/static/images/Superela-1-scaled.jpg.webp"
                                                    />
                                                </div>
                                            </div>
                                            <div className="work-item-description hoverable-md">
                                                <p>
                                                    Where busy women find their
                                                    way
                                                    <br />
                                                    back to themselves
                                                </p>
                                            </div>
                                        </a>
                                    </div>
                                </div>
                            </div>
                        </section>

                        <footer>
                            <div className="container">
                                <div className="footer-row">
                                    <div className="footer-block">
                                        <div className="footer-title">
                                            <h4>문의하기</h4>
                                        </div>
                                        <div className="footer-description">
                                            <p>
                                                새로운 프로젝트가 있으신가요?
                                                <br />
                                                함께 이야기 나눠요.
                                            </p>
                                        </div>
                                        <div className="footer-action">
                                            <a
                                                className="btn-action non-hoverable anchor"
                                                href="/contact"
                                            >
                                                → 문의하기
                                            </a>
                                        </div>
                                    </div>
                                    <div className="footer-block">
                                        <div className="footer-menu">
                                            <ul>
                                                <li>
                                                    <a
                                                        className="hoverable-sm anchor"
                                                        href="/"
                                                    >
                                                        스튜디오
                                                    </a>
                                                </li>
                                                <li>
                                                    <a
                                                        className="hoverable-sm anchor"
                                                        href="/dashboard"
                                                    >
                                                        대시보드
                                                    </a>
                                                </li>
                                                <li>
                                                    <a
                                                        className="hoverable-sm anchor"
                                                        href="/works"
                                                    >
                                                        작업물
                                                    </a>
                                                </li>
                                                <li>
                                                    <a
                                                        className="hoverable-sm anchor"
                                                        href="/contact"
                                                    >
                                                        문의하기
                                                    </a>
                                                </li>
                                            </ul>
                                        </div>
                                    </div>
                                    <div className="footer-block">
                                        <div className="footer-social">
                                            <a
                                                className="social-link hoverable-sm"
                                                href="#"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-instagram"></i>
                                            </a>
                                            <a
                                                className="social-link hoverable-sm"
                                                href="#"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-linkedin-in"></i>
                                            </a>
                                            <a
                                                className="social-link hoverable-sm"
                                                href="#"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-whatsapp"></i>
                                            </a>
                                        </div>
                                    </div>
                                </div>
                                <div className="footer-row">
                                    <div className="footer-block">
                                        <div className="footer-copyright">
                                            <p>
                                                © 2024 Reino Studio. All rights
                                                reserved.
                                            </p>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </footer>
                    </div>
                    <div className="layer layer-back bg-dark"></div>
                </div>
            </div>
        </>
    );
};

export default Works;
