import React, { useEffect, useRef } from "react";
import { ensureLegacyScript } from "../utils/legacyScript";

const Contact = () => {
    const logoWhiteRefs = useRef([]);
    const logoBlackRefs = useRef([]);
    const animationsInitialized = useRef(false);

    useEffect(() => {
        // Set body opacity to 1
        document.body.style.opacity = "1";
        document.body.className = "alternate";

        // Ensure versioned legacy script is available
        ensureLegacyScript();

        // Wait for all libraries to be loaded
        const checkLibraries = setInterval(() => {
            if (window.bodymovin && window.gsap && window.Swiper) {
                clearInterval(checkLibraries);

                // Initialize Lottie animations
                setTimeout(() => {
                    if (!animationsInitialized.current) {
                        animationsInitialized.current = true;

                        // Load white logos
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

            if (window.gsap) {
                window.gsap.killTweensOf("*");
            }
            if (window.ScrollTrigger) {
                window.ScrollTrigger.getAll().forEach((trigger) =>
                    trigger.kill()
                );
            }

            document.body.className = "";
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
                                                className="menu-item anchor hoverable-sm"
                                                href="/works"
                                            >
                                                <span>Works</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm active"
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

                        <section className="section-contact">
                            <div className="container">
                                <div className="contact-content">
                                    <div className="contact-title hoverable">
                                        <h1 className="title">
                                            <span>Hey!</span>
                                            Tell us all the things
                                        </h1>
                                    </div>
                                    <div className="contact-informations">
                                        <div className="contact-address hoverable-sm">
                                            Avenida San Martin. 830, 102
                                            <br />
                                            Leblon - Rio de Janeiro
                                        </div>
                                        <div className="contact-meta">
                                            <div className="meta hoverable-sm">
                                                <span className="meta-icon">
                                                    <i className="fa-regular fa-fw fa-envelope"></i>
                                                </span>
                                                <span className="meta-data">
                                                    <a href="mailto:contact@reinostudio.com">
                                                        contact@reinostudio.com
                                                    </a>
                                                </span>
                                            </div>
                                            <div className="meta hoverable-sm">
                                                <span className="meta-icon">
                                                    <i className="fa-regular fa-fw fa-phone"></i>
                                                </span>
                                                <span className="meta-data">
                                                    <a href="tel:+55 21 98479-6999">
                                                        +55 21 98479-6999
                                                    </a>
                                                </span>
                                            </div>
                                        </div>
                                    </div>
                                    <div className="contact-form">
                                        <div className="wpcf7">
                                            <form
                                                action="/contact/#wpcf7-f9-p27-o1"
                                                className="wpcf7-form init"
                                                method="post"
                                            >
                                                <div className="form-group">
                                                    <div className="form-label">
                                                        I'm interested in...
                                                    </div>
                                                    <div className="form-checkboxes">
                                                        <span
                                                            className="wpcf7-form-control-wrap"
                                                            data-name="interested-topics"
                                                        >
                                                            <span className="wpcf7-form-control wpcf7-checkbox wpcf7-validates-as-required checkbox-inputs">
                                                                <span className="wpcf7-list-item first">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="Site from scratch"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            Site
                                                                            from
                                                                            scratch
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                                <span className="wpcf7-list-item">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="App from scratch"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            App
                                                                            from
                                                                            scratch
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                                <span className="wpcf7-list-item">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="UX/UI design"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            UX/UI
                                                                            design
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                                <span className="wpcf7-list-item">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="Branding"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            Branding
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                                <span className="wpcf7-list-item">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="Animation 2D"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            Animation
                                                                            2D
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                                <span className="wpcf7-list-item">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="Animation 3D"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            Animation
                                                                            3D
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                                <span className="wpcf7-list-item">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="Motion Graphics"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            Motion
                                                                            Graphics
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                                <span className="wpcf7-list-item last">
                                                                    <label>
                                                                        <input
                                                                            name="interested-topics[]"
                                                                            type="checkbox"
                                                                            value="Illustration"
                                                                        />
                                                                        <span className="wpcf7-list-item-label">
                                                                            Illustration
                                                                        </span>
                                                                    </label>
                                                                </span>
                                                            </span>
                                                        </span>
                                                    </div>
                                                </div>
                                                <div className="form-group">
                                                    <div className="floating-field">
                                                        <span
                                                            className="wpcf7-form-control-wrap"
                                                            data-name="fullname"
                                                        >
                                                            <input
                                                                className="wpcf7-form-control wpcf7-text wpcf7-validates-as-required float-input"
                                                                name="fullname"
                                                                size="40"
                                                                type="text"
                                                                maxLength="400"
                                                            />
                                                        </span>
                                                        <div className="float-label">
                                                            Your name
                                                        </div>
                                                    </div>
                                                </div>
                                                <div className="form-group">
                                                    <div className="floating-field">
                                                        <span
                                                            className="wpcf7-form-control-wrap"
                                                            data-name="email"
                                                        >
                                                            <input
                                                                className="wpcf7-form-control wpcf7-email wpcf7-validates-as-required wpcf7-text wpcf7-validates-as-email float-input"
                                                                name="email"
                                                                size="40"
                                                                type="email"
                                                                maxLength="400"
                                                            />
                                                        </span>
                                                        <div className="float-label">
                                                            Your email
                                                        </div>
                                                    </div>
                                                </div>
                                                <div className="form-group">
                                                    <div className="floating-field">
                                                        <span
                                                            className="wpcf7-form-control-wrap"
                                                            data-name="about-project"
                                                        >
                                                            <textarea
                                                                className="wpcf7-form-control wpcf7-textarea wpcf7-validates-as-required float-input"
                                                                name="about-project"
                                                                cols="40"
                                                                rows="10"
                                                                maxLength="2000"
                                                            ></textarea>
                                                        </span>
                                                        <div className="float-label">
                                                            Tell us about your
                                                            project
                                                        </div>
                                                    </div>
                                                </div>
                                                <div className="form-group">
                                                    <div className="btn-submit">
                                                        <input
                                                            className="wpcf7-form-control wpcf7-submit has-spinner input-submit"
                                                            type="submit"
                                                            value="Send Request"
                                                        />
                                                        <span className="wpcf7-spinner"></span>
                                                    </div>
                                                </div>
                                                <div
                                                    className="wpcf7-response-output"
                                                    aria-hidden="true"
                                                ></div>
                                            </form>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </section>

                        <footer>
                            <div className="container">
                                <div className="footer-content">
                                    <div className="footer-information">
                                        <div className="footer-title hoverable">
                                            <a href="/contact">Hello!</a>
                                        </div>
                                        <div className="footer-description hoverable-md">
                                            <p>
                                                <strong>
                                                    Tell us about your project.
                                                </strong>
                                            </p>
                                            <p>
                                                Let's collaborate and make great
                                                stuff.
                                            </p>
                                        </div>
                                    </div>
                                    <div className="footer-contact hoverable-sm">
                                        <span className="contact-icon">
                                            <i className="fa-light fa-fw fa-phone"></i>
                                        </span>
                                        <span className="contact-data">
                                            <a href="tel:+55 21 98479-6999">
                                                +55 21 98479-6999
                                            </a>
                                        </span>
                                    </div>
                                </div>
                                <div className="footer-bar">
                                    <ul className="footer-links">
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/"
                                            >
                                                Studio
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/dashboard"
                                            >
                                                Dashboard
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/works"
                                            >
                                                Works
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/contact"
                                            >
                                                Contact
                                            </a>
                                        </li>
                                    </ul>
                                    <ul className="social-links">
                                        <li>
                                            <a
                                                className="link"
                                                href="https://www.instagram.com/reinostudio/"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-instagram"></i>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link"
                                                href="https://www.linkedin.com/company/reino-studio"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-linkedin-in"></i>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link"
                                                href="https://wa.me/5521984796999"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-whatsapp"></i>
                                            </a>
                                        </li>
                                    </ul>
                                </div>
                            </div>
                        </footer>
                    </div>

                    <div className="layer layer-back">
                        <header>
                            <div className="container">
                                <div className="brand">
                                    <a
                                        className="hoverable-sm anchor brand-logo logo-white"
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
                                                className="menu-item anchor hoverable-sm"
                                                href="/works"
                                            >
                                                <span>Works</span>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="menu-item anchor hoverable-sm active"
                                                href="/contact"
                                            >
                                                <span>Contact</span>
                                            </a>
                                        </li>
                                    </ul>
                                </div>
                            </div>
                        </header>

                        <section className="section-contact">
                            <div className="container">
                                <div className="contact-content">
                                    <div className="contact-title hoverable">
                                        <h1 className="title">
                                            <span>And</span>
                                            We will get in touch
                                        </h1>
                                    </div>
                                    <div className="contact-informations">
                                        <div className="contact-address hoverable-sm">
                                            Avenida San Martin. 830, 102
                                            <br />
                                            Leblon - Rio de Janeiro
                                        </div>
                                        <div className="contact-meta">
                                            <div className="meta hoverable-sm">
                                                <span className="meta-icon">
                                                    <i className="fa-regular fa-fw fa-envelope"></i>
                                                </span>
                                                <span className="meta-data">
                                                    <a href="mailto:contact@reinostudio.com">
                                                        contact@reinostudio.com
                                                    </a>
                                                </span>
                                            </div>
                                            <div className="meta hoverable-sm">
                                                <span className="meta-icon">
                                                    <i className="fa-regular fa-fw fa-phone"></i>
                                                </span>
                                                <span className="meta-data">
                                                    <a href="tel:+55 21 98479-6999">
                                                        +55 21 98479-6999
                                                    </a>
                                                </span>
                                            </div>
                                        </div>
                                    </div>
                                </div>
                            </div>
                        </section>

                        <footer>
                            <div className="container">
                                <div className="footer-content">
                                    <div className="footer-information">
                                        <div className="footer-title hoverable">
                                            Let's Get Started!
                                        </div>
                                        <div className="footer-description hoverable-md">
                                            <p>
                                                <strong>
                                                    Tell us about your project.
                                                </strong>
                                            </p>
                                            <p>
                                                Let's collaborate and make great
                                                stuff.
                                            </p>
                                        </div>
                                    </div>
                                    <div className="footer-contact hoverable-sm">
                                        <span className="contact-icon">
                                            <i className="fa-light fa-fw fa-phone"></i>
                                        </span>
                                        <span className="contact-data">
                                            <a href="tel:+55 21 98479-6999">
                                                +55 21 98479-6999
                                            </a>
                                        </span>
                                    </div>
                                </div>
                                <div className="footer-bar">
                                    <ul className="footer-links">
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/"
                                            >
                                                Studio
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/dashboard"
                                            >
                                                Dashboard
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/works"
                                            >
                                                Works
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link anchor hoverable-sm"
                                                href="/contact"
                                            >
                                                Contact
                                            </a>
                                        </li>
                                    </ul>
                                    <ul className="social-links">
                                        <li>
                                            <a
                                                className="link"
                                                href="https://www.instagram.com/reinostudio/"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-instagram"></i>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link"
                                                href="https://www.linkedin.com/company/reino-studio"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-linkedin-in"></i>
                                            </a>
                                        </li>
                                        <li>
                                            <a
                                                className="link"
                                                href="https://wa.me/5521984796999"
                                                target="_blank"
                                                rel="noopener noreferrer"
                                            >
                                                <i className="fa-brands fa-fw fa-whatsapp"></i>
                                            </a>
                                        </li>
                                    </ul>
                                </div>
                            </div>
                        </footer>
                    </div>
                </div>
            </div>
        </>
    );
};

export default Contact;
