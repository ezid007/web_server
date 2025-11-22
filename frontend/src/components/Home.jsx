import React, { useEffect, useRef } from "react";

const Home = () => {
    const logoWhiteRefs = useRef([]);

    useEffect(() => {
        // Set body opacity
        document.body.style.opacity = "1";

        // Wait for libraries to load
        const initAnimations = () => {
            if (window.bodymovin && window.gsap && window.Swiper) {
                // Load Lottie animations for white logos
                logoWhiteRefs.current.forEach((element) => {
                    if (element && element.children.length === 0) {
                        window.bodymovin.loadAnimation({
                            container: element,
                            path: "/static/json/logo-white.json",
                            renderer: "canvas",
                            loop: false,
                            autoplay: true,
                            name: "Brand Logo White",
                        });
                    }
                });

                // Wait for DOM to be fully painted, then initialize script.js animations
                requestAnimationFrame(() => {
                    requestAnimationFrame(() => {
                        if (typeof window.initializeReinoPage === "function") {
                            console.log("Initializing Reino animations...");
                            window.initializeReinoPage();
                        } else {
                            console.error("initializeReinoPage not found!");
                        }
                    });
                });
            } else {
                // Libraries not ready yet, try again
                setTimeout(initAnimations, 100);
            }
        };

        initAnimations();

        // Cleanup on unmount
        return () => {
            document.body.style.opacity = "0";
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

            <div className="viewport SmoothScroll">
                <div className="main">
                    <div className="layer layer-front">
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
                                                className="menu-item anchor hoverable-sm active"
                                                href="/"
                                            >
                                                <span>스튜디오</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm"
                                                href="/dashboard"
                                            >
                                                <span>대시보드</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm"
                                                href="/works"
                                            >
                                                <span>작업물</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm"
                                                href="/contact"
                                            >
                                                <span>문의하기</span>
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

                        <section className="section-banner">
                            <div className="banner-background">
                                <div className="background-curtain"></div>
                            </div>
                            <div className="container">
                                <div className="banner-content">
                                    <div className="banner-title">
                                        <h1 className="title hoverable">
                                            우리는 정체성,
                                            <br />
                                            경험, 그리고 존재감을 만듭니다.
                                        </h1>
                                    </div>
                                    <div className="banner-sliders">
                                        <div className="banner-slider-row">
                                            <div className="swiper-controls non-hoverable">
                                                <button
                                                    className="swiper-button swiper-button-prev"
                                                    type="button"
                                                >
                                                    <span>
                                                        <i className="fa-solid fa-play fa-flip-horizontal"></i>
                                                    </span>
                                                </button>
                                                <button
                                                    className="swiper-button swiper-button-next"
                                                    type="button"
                                                >
                                                    <span>
                                                        <i className="fa-solid fa-play"></i>
                                                    </span>
                                                </button>
                                            </div>
                                            <div className="banner-slider-project">
                                                <div className="swiper projects-title-1">
                                                    <div className="swiper-wrapper">
                                                        <div className="swiper-slide">
                                                            <div className="project-info">
                                                                <div className="title non-hoverable">
                                                                    GEN
                                                                </div>
                                                                <div className="description non-hoverable">
                                                                    브랜딩,
                                                                    브랜드
                                                                    가이드
                                                                </div>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-info">
                                                                <div className="title non-hoverable">
                                                                    Berrics
                                                                </div>
                                                                <div className="description non-hoverable">
                                                                    전략 &
                                                                    디자인,
                                                                    아이덴티티,
                                                                    인터페이스
                                                                </div>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-info">
                                                                <div className="title non-hoverable">
                                                                    Cariuma
                                                                </div>
                                                                <div className="description non-hoverable">
                                                                    브랜딩,
                                                                    인터랙티브
                                                                    콘텐츠
                                                                </div>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-info">
                                                                <div className="title non-hoverable">
                                                                    Zuso
                                                                </div>
                                                                <div className="description non-hoverable">
                                                                    브랜딩,
                                                                    UX/UI,
                                                                    일러스트레이션,
                                                                    애니메이션
                                                                </div>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-info">
                                                                <div className="title non-hoverable">
                                                                    Superela
                                                                </div>
                                                                <div className="description non-hoverable">
                                                                    앱 디자인,
                                                                    브랜딩,
                                                                    일러스트레이션,
                                                                    모션, UX/UI
                                                                </div>
                                                            </div>
                                                        </div>
                                                    </div>
                                                </div>
                                            </div>
                                        </div>

                                        <div className="banner-slider-row">
                                            <div className="banner-slider-project-primary">
                                                <div className="swiper projects-primary">
                                                    <div className="swiper-wrapper">
                                                        <div className="swiper-slide">
                                                            <div
                                                                className="project-primary-media"
                                                                data-swiper-parallax="50%"
                                                            >
                                                                <a
                                                                    className="project-link anchor"
                                                                    href="/works/gen/"
                                                                >
                                                                    <div className="project-image image-hover">
                                                                        <img
                                                                            alt="GEN"
                                                                            src="/static/images/Gen-01-scaled.jpg.webp"
                                                                        />
                                                                    </div>
                                                                    <div className="project-details">
                                                                        <h2 className="title">
                                                                            GEN
                                                                        </h2>
                                                                        <div className="category">
                                                                            브랜딩,
                                                                            브랜드
                                                                            가이드
                                                                        </div>
                                                                        <div className="location">
                                                                            미국
                                                                            애틀랜타
                                                                        </div>
                                                                    </div>
                                                                </a>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div
                                                                className="project-primary-media"
                                                                data-swiper-parallax="50%"
                                                            >
                                                                <a
                                                                    className="project-link anchor"
                                                                    href="/works/berrics/"
                                                                >
                                                                    <div className="project-image image-hover">
                                                                        <img
                                                                            alt="Berrics"
                                                                            src="/static/images/Berrics-1-scaled.jpg.webp"
                                                                        />
                                                                    </div>
                                                                    <div className="project-details">
                                                                        <h2 className="title">
                                                                            Berrics
                                                                        </h2>
                                                                        <div className="category">
                                                                            전략
                                                                            &
                                                                            디자인,
                                                                            아이덴티티,
                                                                            인터페이스
                                                                        </div>
                                                                        <div className="location">
                                                                            미국
                                                                            로스앤젤레스
                                                                        </div>
                                                                    </div>
                                                                </a>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div
                                                                className="project-primary-media"
                                                                data-swiper-parallax="50%"
                                                            >
                                                                <a
                                                                    className="project-link anchor"
                                                                    href="/works/cariuma/"
                                                                >
                                                                    <div className="project-image image-hover">
                                                                        <img
                                                                            alt="Cariuma"
                                                                            src="/static/images/Cariuma-1-1-scaled.jpg.webp"
                                                                        />
                                                                    </div>
                                                                    <div className="project-details">
                                                                        <h2 className="title">
                                                                            Cariuma
                                                                        </h2>
                                                                        <div className="category">
                                                                            브랜딩,
                                                                            인터랙티브
                                                                            콘텐츠
                                                                        </div>
                                                                        <div className="location">
                                                                            싱가포르
                                                                        </div>
                                                                    </div>
                                                                </a>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div
                                                                className="project-primary-media"
                                                                data-swiper-parallax="50%"
                                                            >
                                                                <a
                                                                    className="project-link anchor"
                                                                    href="/works/zuso/"
                                                                >
                                                                    <div className="project-video non-hoverable">
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
                                                                    <div className="project-details">
                                                                        <h2 className="title">
                                                                            Zuso
                                                                        </h2>
                                                                        <div className="category">
                                                                            브랜딩,
                                                                            UX/UI,
                                                                            일러스트레이션,
                                                                            애니메이션
                                                                        </div>
                                                                        <div className="location">
                                                                            미국
                                                                            솔트레이크시티
                                                                        </div>
                                                                    </div>
                                                                </a>
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div
                                                                className="project-primary-media"
                                                                data-swiper-parallax="50%"
                                                            >
                                                                <a
                                                                    className="project-link anchor"
                                                                    href="/works/superela/"
                                                                >
                                                                    <div className="project-image image-hover">
                                                                        <img
                                                                            alt="Superela"
                                                                            src="/static/images/Superela-1-scaled.jpg.webp"
                                                                        />
                                                                    </div>
                                                                    <div className="project-details">
                                                                        <h2 className="title">
                                                                            Superela
                                                                        </h2>
                                                                        <div className="category">
                                                                            앱
                                                                            디자인,
                                                                            브랜딩,
                                                                            일러스트레이션,
                                                                            모션,
                                                                            UX/UI
                                                                        </div>
                                                                        <div className="location">
                                                                            포르투갈
                                                                            카스카이스
                                                                        </div>
                                                                    </div>
                                                                </a>
                                                            </div>
                                                        </div>
                                                    </div>
                                                </div>
                                            </div>

                                            <div className="web-information">
                                                <div className="action">
                                                    <a
                                                        className="btn-action non-hoverable"
                                                        href="/works"
                                                    >
                                                        ← 작업물 보기
                                                    </a>
                                                </div>
                                                <div className="options">
                                                    <div className="social non-hoverable">
                                                        <a
                                                            className="social-link"
                                                            href="#"
                                                            title="Instagram"
                                                            target="_blank"
                                                            rel="noreferrer"
                                                        >
                                                            <i className="fa-brands fa-fw fa-instagram"></i>
                                                        </a>
                                                        <a
                                                            className="social-link"
                                                            href="#"
                                                            title="LinkedIn"
                                                            target="_blank"
                                                            rel="noreferrer"
                                                        >
                                                            <i className="fa-brands fa-fw fa-linkedin-in"></i>
                                                        </a>
                                                        <a
                                                            className="social-link"
                                                            href="#"
                                                            title="WhatsApp"
                                                            target="_blank"
                                                            rel="noreferrer"
                                                        >
                                                            <i className="fa-brands fa-fw fa-whatsapp"></i>
                                                        </a>
                                                    </div>
                                                    <div className="scroll-action non-hoverable">
                                                        <button
                                                            className="scroll-down-button"
                                                            type="button"
                                                        >
                                                            <span>
                                                                <i className="fa-solid fa-play fa-rotate-90"></i>
                                                            </span>
                                                        </button>
                                                    </div>
                                                </div>
                                            </div>
                                        </div>

                                        <div
                                            className="banner-slider-row"
                                            id="scrollTo"
                                        >
                                            <div className="banner-slider-project-wrapper">
                                                <div className="banner-slider-project-secondary">
                                                    <div className="swiper projects-secondary-1">
                                                        <div className="swiper-wrapper">
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/gen/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="GEN"
                                                                                src="/static/images/Gen-32-1.png.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/berrics/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="Berrics"
                                                                                src="/static/images/Berrics-23.jpg.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/cariuma/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="Cariuma"
                                                                                src="/static/images/Cariuma-12.jpg.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/zuso/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="Zuso"
                                                                                src="/static/images/Zuso-11.jpg.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/superela/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="Superela"
                                                                                src="/static/images/Superela-8.jpg.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                        </div>
                                                    </div>
                                                </div>

                                                <div className="banner-slider-project-secondary">
                                                    <div className="swiper projects-secondary-2">
                                                        <div className="swiper-wrapper">
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/gen/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="GEN"
                                                                                src="/static/images/Gen-21-1.png.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/berrics/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="Berrics"
                                                                                src="/static/images/Berrics-22.jpg.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/cariuma/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="Cariuma"
                                                                                src="/static/images/Cariuma-11.jpg.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/zuso/"
                                                                    >
                                                                        <div className="video non-hoverable">
                                                                            <video
                                                                                autoPlay
                                                                                loop
                                                                                muted
                                                                                playsInline
                                                                            >
                                                                                <source
                                                                                    src="/static/videos/Weekly_Planner.mp4"
                                                                                    type="video/mp4"
                                                                                />
                                                                            </video>
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                            <div className="swiper-slide">
                                                                <div
                                                                    className="project-secondary"
                                                                    data-swiper-parallax="50%"
                                                                >
                                                                    <a
                                                                        className="anchor"
                                                                        href="/works/superela/"
                                                                    >
                                                                        <div className="image image-hover">
                                                                            <img
                                                                                alt="Superela"
                                                                                src="/static/images/Superela-23.jpg.webp"
                                                                            />
                                                                        </div>
                                                                    </a>
                                                                </div>
                                                            </div>
                                                        </div>
                                                    </div>
                                                </div>
                                            </div>
                                        </div>

                                        <div className="banner-slider-row">
                                            <div className="swiper-controls alternate non-hoverable">
                                                <button
                                                    className="swiper-button swiper-button-prev"
                                                    type="button"
                                                >
                                                    <span>
                                                        <i className="fa-solid fa-play fa-flip-horizontal"></i>
                                                    </span>
                                                </button>
                                                <button
                                                    className="swiper-button swiper-button-next"
                                                    type="button"
                                                >
                                                    <span>
                                                        <i className="fa-solid fa-play"></i>
                                                    </span>
                                                </button>
                                            </div>
                                            <div className="banner-slider-project alternate">
                                                <div className="swiper projects-title-2">
                                                    <div className="swiper-wrapper">
                                                        <div className="swiper-slide">
                                                            <div className="project-name non-hoverable">
                                                                GEN
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-name non-hoverable">
                                                                Berrics
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-name non-hoverable">
                                                                Cariuma
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-name non-hoverable">
                                                                Zuso
                                                            </div>
                                                        </div>
                                                        <div className="swiper-slide">
                                                            <div className="project-name non-hoverable">
                                                                Superela
                                                            </div>
                                                        </div>
                                                    </div>
                                                </div>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </section>
                    </div>
                </div>
            </div>
        </>
    );
};

export default Home;
