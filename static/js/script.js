// 전역 설정
var windowWidth =
    window.innerWidth ||
    document.documentElement.clientWidth ||
    document.body.clientWidth;
var body = document.querySelector("body");
var viewport = document.querySelector(".viewport");
var layerFront = document.querySelector(".layer-front");
var layerBack = document.querySelector(".layer-back");
var hoverable = document.querySelectorAll(".hoverable");
var hoverableMedium = document.querySelectorAll(".hoverable-md");
var hoverableSmall = document.querySelectorAll(".hoverable-sm");
var nonHoverable = document.querySelectorAll(".non-hoverable");
var brandLogoWhite = document.querySelectorAll(".logo-white");
var brandLogoBlack = document.querySelectorAll(".logo-black");
var navigationMenus = document.querySelectorAll(".navigation-menu");
var btnMenuTrigger = document.querySelectorAll(".btn-menu-trigger");
var imageHover = document.querySelectorAll(".image-hover");
var footers = document.querySelectorAll("footer");
var frontFooter = document.querySelectorAll(".layer-front footer");
var bannerTitle = document.querySelectorAll(".banner-title .title");
var bannerHighlight = document.querySelectorAll(".banner-title .highlight");
var backgroundCurtain = document.querySelectorAll(
    ".banner-background .background-curtain.animation"
);
var informations = document.querySelectorAll(".project-banner .information");
var bannerProjectImage = document.querySelector(
    ".project-banner .project-image"
);
var projectsTitleOne = document.querySelector(".projects-title-1");
var projectsTitleTwo = document.querySelector(".projects-title-2");
var webInformationAction = document.querySelector(".web-information .action");
var webInformationOption = document.querySelector(".web-information .options");
var projectsPrimary = document.querySelector(".projects-primary");
var projectsSecondary1 = document.querySelector(".projects-secondary-1");
var projectsSecondary2 = document.querySelector(".projects-secondary-2");
var swiperControl = document.querySelectorAll(".swiper-controls");
var swiperControl = document.querySelectorAll(".swiper-controls");
var swiperButton = document.querySelectorAll(".swiper-button");
var swiperButtonAlternate = document.querySelectorAll(
    ".alternate .swiper-button"
);
var scrollDownButton = document.querySelector(".scroll-down-button");
var scrollToSection = document.querySelector("#scrollTo");
var aboutContainers = document.querySelectorAll(".section-about");
var serviceContainer = document.querySelector(".section-services");
var clientContainer = document.querySelector(".section-clients");
var clientListSwiper = document.querySelector(".client-list");
var projectDetailsContainer = document.querySelector(
    ".section-project-details"
);
var contactTitle = document.querySelectorAll(".section-contact .contact-title");
var contactInformations = document.querySelectorAll(
    ".section-contact .contact-informations"
);
var contactForm = document.querySelector(".section-contact .contact-form");
var projectMediaGridBlock3inC = document.querySelectorAll(
    ".swiper.project-media-grid-block.type-3inC"
);
var projectMediaGridBlock2inC = document.querySelectorAll(
    ".swiper.project-media-grid-block.type-2inC"
);
var anchor = document.querySelectorAll(".anchor");
var homeUrlInput = document.querySelector("#home_url");
var revealSlideUpItems = document.querySelectorAll(".reveal-slide-up");
var revealFadeItems = document.querySelectorAll(".reveal-fade");
var reinoGlobalEventsBound = false;

function cacheDomElements() {
    windowWidth =
        window.innerWidth ||
        document.documentElement.clientWidth ||
        document.body.clientWidth;
    body = document.querySelector("body");
    viewport = document.querySelector(".viewport");
    layerFront = document.querySelector(".layer-front");
    layerBack = document.querySelector(".layer-back");
    hoverable = document.querySelectorAll(".hoverable");
    hoverableMedium = document.querySelectorAll(".hoverable-md");
    hoverableSmall = document.querySelectorAll(".hoverable-sm");
    nonHoverable = document.querySelectorAll(".non-hoverable");
    brandLogoWhite = document.querySelectorAll(".logo-white");
    brandLogoBlack = document.querySelectorAll(".logo-black");
    navigationMenus = document.querySelectorAll(".navigation-menu");
    btnMenuTrigger = document.querySelectorAll(".btn-menu-trigger");
    imageHover = document.querySelectorAll(".image-hover");
    footers = document.querySelectorAll("footer");
    frontFooter = document.querySelectorAll(".layer-front footer");
    bannerTitle = document.querySelectorAll(".banner-title .title");
    bannerHighlight = document.querySelectorAll(".banner-title .highlight");
    backgroundCurtain = document.querySelectorAll(
        ".banner-background .background-curtain.animation"
    );
    informations = document.querySelectorAll(".project-banner .information");
    bannerProjectImage = document.querySelector(
        ".project-banner .project-image"
    );
    projectsTitleOne = document.querySelector(".projects-title-1");
    projectsTitleTwo = document.querySelector(".projects-title-2");
    webInformationAction = document.querySelector(".web-information .action");
    webInformationOption = document.querySelector(".web-information .options");
    projectsPrimary = document.querySelector(".projects-primary");
    projectsSecondary1 = document.querySelector(".projects-secondary-1");
    projectsSecondary2 = document.querySelector(".projects-secondary-2");
    swiperControl = document.querySelectorAll(".swiper-controls");
    swiperButton = document.querySelectorAll(".swiper-button");
    swiperButtonAlternate = document.querySelectorAll(
        ".alternate .swiper-button"
    );
    scrollDownButton = document.querySelector(".scroll-down-button");
    scrollToSection = document.querySelector("#scrollTo");
    aboutContainers = document.querySelectorAll(".section-about");
    serviceContainer = document.querySelector(".section-services");
    clientContainer = document.querySelector(".section-clients");
    clientListSwiper = document.querySelector(".client-list");
    projectDetailsContainer = document.querySelector(
        ".section-project-details"
    );
    contactTitle = document.querySelectorAll(".section-contact .contact-title");
    contactInformations = document.querySelectorAll(
        ".section-contact .contact-informations"
    );
    contactForm = document.querySelector(".section-contact .contact-form");
    projectMediaGridBlock3inC = document.querySelectorAll(
        ".swiper.project-media-grid-block.type-3inC"
    );
    projectMediaGridBlock2inC = document.querySelectorAll(
        ".swiper.project-media-grid-block.type-2inC"
    );
    anchor = document.querySelectorAll(".anchor");
    homeUrlInput = document.querySelector("#home_url");
    revealSlideUpItems = document.querySelectorAll(".reveal-slide-up");
    revealFadeItems = document.querySelectorAll(".reveal-fade");
}

var mouseX = 0;
var mouseY = 0;
var currentMouseX = 0;
var currentMouseY = 0;

const isTouchDevice = "ontouchstart" in document.documentElement;

// 터치 디바이스 처리
if (isTouchDevice) {
    layerBack.remove();
}

// GSAP 뷰포트 로드 지연
function vieportLoad() {
    if (viewport) {
        gsap.from(viewport, {
            opacity: 0,
            duration: 0.5,
            delay: 0.5,
            ease: "sine.out",
            //onComplete: startNextAnimation
        });
    }
}

// GSAP 푸터 배경 스크롤 트리거
function pageFooterFade() {
    gsap.registerPlugin(ScrollTrigger);
    if (frontFooter && layerFront) {
        gsap.to(layerFront, {
            scrollTrigger: {
                trigger: frontFooter,
                start: "top center",
                end: "bottom center",
                scrub: 1,
            },
            background: "#000000",
            ease: "power4.out",
            duration: 1,
            delay: 0,
        });
        ScrollTrigger.refresh();
    }
}

// 부드러운 스크롤
if (!isTouchDevice) {
    disableScroll();
}
window.onresize = () => {
    resizeBodyHeight();
};
window.onload = () => {
    resizeBodyHeight();
    enableScroll();
    setTimeout(() => {
        if (!isTouchDevice) {
            smoothScroll();
        }
    }, [300]);
};
function disableScroll() {
    if (body) {
        body.style.overflow = "hidden";
    }
}
function enableScroll() {
    document.body.style.overflow = "";
}
function smoothScroll() {
    var viewports = document.querySelector(".viewport");
    viewports.classList.add("SmoothScroll");
    var holder = viewports.querySelector(".main");
    new SmoothScroll({
        target: holder,
        scrollEase: 0.08,
        maxOffset: 500,
        depth: 0,
    });
}
function resizeBodyHeight() {
    var viewports = document.querySelectorAll(".viewport");
    viewports.forEach((viewport) => {
        document.body.style.height = viewport.scrollHeight + "px";
    });
}

// 앵커 클릭 시 페이드 효과
function anchorRedirection() {
    if (anchor && viewport) {
        anchor.forEach((element) => {
            element.addEventListener("click", (event) => {
                event.preventDefault();
                var link = element.getAttribute("href");
                gsap.to(viewport, {
                    opacity: 0,
                    duration: 0.5,
                    delay: 0,
                    ease: "sine.out",
                });
                setTimeout(
                    function () {
                        window.location.href = link;
                    },
                    [500]
                );
            });
        });
    }
}

// 뒤로 가기 버튼 클릭 시 페이지 새로고침
window.onpageshow = function (event) {
    if (event.persisted) {
        window.location.reload();
    }
};

// 전면 및 후면 레이어 섹션 높이 동일화 함수
function logDivHeight(source, target) {
    var sourceElement = document.querySelector(source);
    var targetElement = document.querySelector(target);
    if (sourceElement) {
        var rect = sourceElement.getBoundingClientRect();
    }
    if (targetElement && rect) {
        targetElement.style.height = rect.height + "px";
    }
}

// Lottie 애니메이션 초기화
function loadBrandLogoWhite() {
    brandLogoWhite.forEach(function (brandLogo) {
        brandLogo.innerHTML = "";
        var brandWhite = bodymovin.loadAnimation({
            container: brandLogo,
            path: "/static/json/logo-white.json",
            renderer: "canvas",
            loop: false,
            autoplay: true,
            name: "Brand Animation",
        });
    });
}
function loadBrandLogoBlack() {
    brandLogoBlack.forEach(function (brandLogo) {
        brandLogo.innerHTML = "";
        var brandBlack = bodymovin.loadAnimation({
            container: brandLogo,
            path: "/static/json/logo.json",
            renderer: "canvas",
            loop: false,
            autoplay: true,
            name: "Brand Animation",
        });
    });
}

function initializeReinoPage() {
    "use strict";

    // React SPA 환경에서 페이지 전환 시 기존 애니메이션 완전 정리
    if (window.ScrollTrigger) {
        window.ScrollTrigger.getAll().forEach((trigger) => trigger.kill());
        window.ScrollTrigger.refresh();
    }

    // 기존 GSAP 애니메이션 모두 제거
    if (window.gsap) {
        window.gsap.killTweensOf("*");
    }

    cacheDomElements();

    //vieportLoad();
    anchorRedirection();
    setTimeout(() => {
        loadBrandLogoWhite();
        loadBrandLogoBlack();
    }, [1000]);

    // GSAP 타임라인
    const timeline = gsap.timeline();

    if (!reinoGlobalEventsBound) {
        // GSAP 커서
        window.addEventListener("mousemove", function (e) {
            mouseX = e.pageX;
            mouseY = e.pageY;
            currentMouseX = e.clientX;
            currentMouseY = e.clientY;
            if (layerBack) {
                gsap.to(layerBack, {
                    "--x": `${mouseX}px`,
                    "--y": `${mouseY}px`,
                    duration: 0.5,
                    stagger: 0,
                    ease: "sine.out",
                });
            }
        });

        // 스크롤 시 커서 위치
        window.addEventListener("scroll", function (e) {
            var scrollX = window.scrollX || window.pageXOffset;
            var scrollY = window.scrollY || window.pageYOffset;
            if (layerBack) {
                gsap.to(layerBack, {
                    "--x": `${scrollX + currentMouseX}px`,
                    "--y": `${scrollY + currentMouseY}px`,
                });
            }
        });

        reinoGlobalEventsBound = true;
    }

    // GSAP 호버 효과
    if (hoverable && layerBack) {
        var windowWidth =
            window.innerWidth ||
            document.documentElement.clientWidth ||
            document.body.clientWidth;
        var size = "500px";
        if (windowWidth > 1024 && windowWidth <= 1440) {
            size = "400px";
        } else if (windowWidth <= 1024) {
            size = "300px";
        }
        hoverable.forEach((element) => {
            if (element.dataset.reinoHoverBound === "true") {
                return;
            }
            element.dataset.reinoHoverBound = "true";
            element.addEventListener("mousemove", () => {
                gsap.to(layerBack, {
                    "--size": size,
                    duration: 0.2,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
            element.addEventListener("mouseleave", () => {
                gsap.to(layerBack, {
                    "--size": `30px`,
                    duration: 0.5,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
        });
    }

    // GSAP 호버 효과 중간 크기
    if (hoverableMedium && layerBack) {
        hoverableMedium.forEach((element) => {
            element.addEventListener("mousemove", () => {
                gsap.to(layerBack, {
                    "--size": `160px`,
                    duration: 0.2,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
            element.addEventListener("mouseleave", () => {
                gsap.to(layerBack, {
                    "--size": `30px`,
                    duration: 0.5,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
        });
    }

    // GSAP 호버 효과 작은 크기
    if (hoverableSmall && layerBack) {
        hoverableSmall.forEach((element) => {
            element.addEventListener("mousemove", () => {
                gsap.to(layerBack, {
                    "--size": `80px`,
                    duration: 0.2,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
            element.addEventListener("mouseleave", () => {
                gsap.to(layerBack, {
                    "--size": `30px`,
                    duration: 0.5,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
        });
    }

    // GSAP 호버 불가
    if (nonHoverable && layerBack) {
        nonHoverable.forEach((element) => {
            element.addEventListener("mousemove", () => {
                gsap.to(layerBack, {
                    "--size": `0px`,
                    duration: 0.2,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
            element.addEventListener("mouseleave", () => {
                gsap.to(layerBack, {
                    "--size": `30px`,
                    duration: 0.5,
                    stagger: 0.01,
                    ease: "sine.out",
                });
            });
        });
    }

    // GSAP 이미지 호버 효과
    if (imageHover) {
        imageHover.forEach((element) => {
            var image = element.querySelector("img");
            element.addEventListener("mouseenter", () => {
                gsap.to(image, {
                    scale: 1.05,
                    duration: 0.5,
                    ease: "sine.out",
                });
            });
            element.addEventListener("mouseleave", () => {
                gsap.to(image, {
                    scale: 1,
                    duration: 0.5,
                    ease: "sine.out",
                });
            });
        });
    }

    // GSAP 메뉴 아이템 타임라인
    if (navigationMenus) {
        navigationMenus.forEach((navigationMenu) => {
            var menuItems =
                navigationMenu.querySelectorAll("li .menu-item span");
            gsap.to(menuItems, {
                duration: 0.5,
                y: "0%",
                ease: "power4.out",
                delay: 1,
                stagger: 0.2,
            });
        });
    }

    // GSAP 메뉴 토글
    if (btnMenuTrigger && navigationMenus && layerBack) {
        var menuOpen = 0;
        var menuContainer = document.querySelectorAll(".navigation");
        var navigationHeader = document.querySelectorAll(".navigation-header");
        btnMenuTrigger.forEach((btnMenu) => {
            btnMenu.addEventListener("click", function () {
                if (menuOpen === 0) {
                    menuOpen = 1;
                    gsap.to(menuContainer[0], {
                        duration: 0.5,
                        opacity: 1,
                        visibility: "visible",
                        y: "0%",
                        ease: "power4.out",
                    });
                    gsap.to(navigationHeader[0], {
                        delay: 0.5,
                        duration: 0.5,
                        opacity: 1,
                        visibility: "visible",
                        ease: "sine.out",
                    });
                    var menuItems =
                        menuContainer[0].querySelectorAll("li .menu-item span");
                    menuItems.forEach((element, index) => {
                        gsap.from(element, {
                            duration: 0.5,
                            y: 80,
                            opacity: 0,
                            ease: "sine.out",
                            delay: 0.5 * index,
                            stagger: {
                                amount: 2,
                            },
                        });
                    });
                    gsap.to(layerBack, {
                        "--size": 0,
                        duration: 0.2,
                        stagger: 0.01,
                        ease: "sine.out",
                    });
                } else {
                    menuOpen = 0;
                    gsap.to(navigationHeader[0], {
                        duration: 0.5,
                        opacity: 0,
                        visibility: "invisible",
                        ease: "sine.out",
                    });
                    var menuItems =
                        menuContainer[0].querySelectorAll("li .menu-item span");
                    menuItems.forEach((element, index) => {
                        gsap.to(element, {
                            duration: 0.5,
                            opacity: 0,
                            ease: "sine.out",
                            delay: 0.1 * index,
                        });
                    });
                    gsap.to(menuContainer[0], {
                        delay: 0.5,
                        duration: 0.5,
                        opacity: 0,
                        visibility: "invisible",
                        y: "100%",
                        ease: "power4.in",
                    });
                    gsap.to(layerBack, {
                        "--size": 30,
                        duration: 0.2,
                        stagger: 0.01,
                        ease: "sine.out",
                    });
                    setTimeout(() => {
                        menuItems.forEach((element, index) => {
                            gsap.to(element, {
                                duration: 0.1,
                                opacity: 1,
                                ease: "sine.out",
                                delay: 0.1,
                            });
                        });
                    }, [1000]);
                }
            });
        });
    }

    // GSAP 배너 커튼 타임라인
    if (backgroundCurtain) {
        var backgroundCurtains = backgroundCurtain.forEach((element) => {
            gsap.from(element, {
                scaleY: 0,
                ease: "power4.out",
                duration: 1.6,
                delay: 0.5,
                stagger: {
                    amount: 1,
                },
            });
        });
    }

    // GSAP 배너 제목 타임라인
    if (bannerTitle) {
        var bannerTitles = bannerTitle.forEach((element) => {
            gsap.from(element, {
                y: 80,
                opacity: 0,
                ease: "power4.out",
                duration: 1,
                delay: 1,
                skewY: 5,
                stagger: {
                    amount: 1,
                },
            });
        });
    }

    // GSAP 배너 강조 타임라인
    if (bannerHighlight) {
        var bannerHighlights = bannerHighlight.forEach((element) => {
            gsap.from(element, {
                y: 80,
                opacity: 0,
                ease: "power3.out",
                duration: 1.5,
                delay: 0.2,
            });
        });
    }

    // GSAP 배너 정보 타임라인
    if (informations.length > 0) {
        gsap.from(informations, {
            y: 80,
            opacity: 0,
            ease: "power3.out",
            duration: 1,
            delay: 1,
            stagger: 0.2,
        });
    }

    // GSAP 배너 이미지 타임라인
    if (bannerProjectImage) {
        var projectImage = gsap.from(bannerProjectImage, {
            y: "50%",
            opacity: 0,
            ease: "sine.out",
            duration: 1,
            delay: 0.5,
        });
    }

    // GSAP 배너 슬라이더 타임라인
    if (
        projectsPrimary &&
        projectsSecondary1 &&
        projectsSecondary2 &&
        swiperControl &&
        projectsTitleOne &&
        projectsTitleTwo &&
        webInformationAction &&
        webInformationOption &&
        layerBack
    ) {
        if (projectsPrimary) {
            var projectsPrimarySlider = gsap.from(projectsPrimary, {
                y: "100%",
                ease: "power4.out",
                duration: 1,
                delay: 0.5,
            });
        }
        if (projectsSecondary1) {
            var projectsSecondary1Slider = gsap.from(projectsSecondary1, {
                y: "100%",
                ease: "power4.out",
                duration: 1,
                delay: 1,
            });
        }
        if (projectsSecondary2) {
            var projectsSecondary2Slider = gsap.from(projectsSecondary2, {
                y: "100%",
                ease: "power4.out",
                duration: 1,
                delay: 0.75,
            });
        }
        if (swiperControl) {
            var swiperControls = swiperControl.forEach((element) => {
                gsap.from(element, {
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 1.5,
                });
            });
        }
        if (projectsTitleOne) {
            gsap.from(projectsTitleOne, {
                opacity: 0,
                ease: "power4.out",
                duration: 0.5,
                delay: 2,
            });
        }
        if (projectsTitleTwo) {
            gsap.from(projectsTitleTwo, {
                opacity: 0,
                ease: "power4.out",
                duration: 0.5,
                delay: 2,
            });
        }
        if (webInformationAction) {
            gsap.from(webInformationAction, {
                opacity: 0,
                ease: "power4.out",
                duration: 0.5,
                delay: 2,
            });
        }
        if (webInformationOption) {
            gsap.from(webInformationOption, {
                opacity: 0,
                ease: "power4.out",
                duration: 0.5,
                delay: 1.5,
            });
        }
        //timeline.timeScale(1.8);
        timeline
            .add([
                projectsPrimarySlider,
                projectsSecondary1Slider,
                projectsSecondary2Slider,
            ])
            .add(swiperControls)
            .add(projectsTitleOne);
    }

    // GSAP 스와이퍼 버튼
    if (swiperButton && layerBack) {
        swiperButton.forEach((element) => {
            var arrow = element.querySelector("span");
            element.addEventListener("mouseenter", () => {
                gsap.to(arrow, {
                    scale: 1,
                    color: "#000",
                    duration: 0.3,
                    ease: "power3.out",
                });
            });
            element.addEventListener("mouseleave", () => {
                gsap.to(arrow, {
                    scale: 0.2,
                    color: "#fff",
                    duration: 0.3,
                    ease: "power3.in",
                });
            });
        });
    }

    // GSAP 스와이퍼 버튼 대체
    if (swiperButtonAlternate && layerBack) {
        swiperButtonAlternate.forEach((element) => {
            var arrow = element.querySelector("span");
            var windowWidth =
                window.innerWidth ||
                document.documentElement.clientWidth ||
                document.body.clientWidth;
            element.addEventListener("mouseenter", () => {
                gsap.to(arrow, {
                    scale: 1,
                    color: "#fff",
                    duration: 0.3,
                    ease: "power3.out",
                });
            });
            element.addEventListener("mouseleave", () => {
                gsap.to(arrow, {
                    scale: 0.2,
                    color: "#000",
                    duration: 0.3,
                    ease: "power3.in",
                });
            });
        });
    }

    // GSAP 아래로 스크롤 버튼
    if (scrollDownButton && layerBack) {
        var arrow = scrollDownButton.querySelector("span");
        scrollDownButton.addEventListener("mouseenter", () => {
            gsap.to(arrow, {
                scale: 1,
                color: "#fff",
                duration: 0.3,
                ease: "power3.out",
            });
        });
        scrollDownButton.addEventListener("mouseleave", () => {
            gsap.to(arrow, {
                scale: 0.2,
                color: "#000",
                duration: 0.3,
                ease: "power3.in",
            });
        });
    }

    // GSAP 섹션 스크롤
    if (scrollToSection && scrollDownButton) {
        scrollDownButton.addEventListener("click", function () {
            console.log("clicked");
            gsap.registerPlugin(ScrollToPlugin);
            gsap.to(window, {
                duration: 0.6,
                scrollTo: { y: scrollToSection, offsetY: 50 },
            });
        });
    }

    // GSAP 소개 스크롤 트리거
    if (aboutContainers && windowWidth > 480) {
        aboutContainers.forEach((aboutContainer) => {
            var aboutTitle = aboutContainer.querySelector(
                ".about-title .title"
            );
            var aboutDescription =
                aboutContainer.querySelectorAll(".about-description");
            gsap.registerPlugin(ScrollTrigger);
            gsap.from(aboutTitle, {
                scrollTrigger: {
                    trigger: aboutContainer,
                    start: "top center",
                    end: "bottom center",
                    scrub: false,
                    once: true,
                },
                y: "200%",
                opacity: 0,
                ease: "power4.out",
                duration: 1,
            });
            aboutDescription.forEach((item) => {
                gsap.from(item, {
                    scrollTrigger: {
                        trigger: aboutContainer,
                        start: "top center",
                        end: "bottom center",
                        scrub: false,
                        once: true,
                    },
                    y: "100%",
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 1,
                });
            });
        });
    }

    //GSAP 서비스 스크롤 트리거
    if (serviceContainer && windowWidth > 480) {
        var serviceItem = serviceContainer.querySelectorAll(".service-item");
        // 서비스 제목
        var serviceTitle = serviceContainer.querySelector(
            ".service-title .title"
        );
        gsap.registerPlugin(ScrollTrigger);
        gsap.from(serviceTitle, {
            scrollTrigger: {
                trigger: serviceContainer,
                start: "top center",
                end: "bottom center",
                scrub: false,
                once: true,
            },
            y: "200%",
            opacity: 0,
            ease: "power4.out",
            duration: 0.5,
        });
        // 서비스 아이템
        serviceItem.forEach((item, index) => {
            var title = item.querySelector(".item-front .item-title");
            gsap.from(title, {
                scrollTrigger: {
                    trigger: serviceContainer,
                    start: "top center",
                    end: "bottom center",
                    scrub: false,
                    once: true,
                },
                y: "100%",
                opacity: 0,
                ease: "power4.out",
                duration: 0.5,
                delay: 0.4 * index,
            });
            var image = item.querySelector(".item-front img");
            var windowWidth =
                window.innerWidth ||
                document.documentElement.clientWidth ||
                document.body.clientWidth;
            var imageWidth = 100;
            if (windowWidth > 1366 && windowWidth <= 1460) {
                imageWidth = 96;
            } else if (windowWidth > 1024 && windowWidth <= 1366) {
                imageWidth = 80;
            } else if (windowWidth > 990 && windowWidth <= 1024) {
                imageWidth = 64;
            } else if (windowWidth <= 990) {
                imageWidth = 48;
            }
            gsap.from(image, {
                scrollTrigger: {
                    trigger: serviceContainer,
                    start: "top center",
                    end: "bottom center",
                    scrub: false,
                    once: true,
                },
                width: imageWidth,
                ease: "power4.out",
                duration: 0.5,
                delay: 1,
                onComplete: function () {
                    image.style.width = "100%";
                },
            });
        });
    }

    // GSAP 마퀴 텍스트
    const marquee = roll(".marquee-text", { duration: 16 });
    gsap.to(marquee, { timeScale: 1, overwrite: false });
    function roll(targets, vars, reverse) {
        const tl = gsap.timeline({
            repeat: -1,
            onReverseComplete() {
                this.totalTime(this.rawTime() + this.duration() * 10);
            },
        });
        vars = vars || {};
        vars.ease || (vars.ease = "none");
        gsap.utils.toArray(targets).forEach((el) => {
            let clone = el.cloneNode(true);
            el.parentNode.appendChild(clone);
            gsap.set(clone, {
                position: "absolute",
                top: el.offsetTop,
                left:
                    el.offsetLeft +
                    (reverse ? -el.offsetWidth : el.offsetWidth),
            });
            tl.to([el, clone], { xPercent: reverse ? 100 : -100, ...vars }, 0);
        });
        return tl;
    }

    // GSAP 클라이언트 스크롤 트리거
    if (clientContainer && windowWidth > 480) {
        var client = clientContainer.querySelectorAll(".client");
        gsap.registerPlugin(ScrollTrigger);
        client.forEach((item, index) => {
            gsap.from(item, {
                scrollTrigger: {
                    trigger: clientContainer,
                    start: "top center",
                    end: "bottom center",
                    scrub: false,
                    once: true,
                },
                y: "100%",
                opacity: 0,
                ease: "power4.out",
                duration: 0.5,
                delay: 0.1 * index,
            });
        });
    }

    // GSAP 푸터 스크롤 트리거
    if (footers) {
        gsap.registerPlugin(ScrollTrigger);
        footers.forEach((footer) => {
            // 푸터 제목
            var footerTitle = footer.querySelector(".footer-title");
            if (footerTitle) {
                gsap.from(footerTitle, {
                    scrollTrigger: {
                        trigger: footer,
                        start: "top center",
                        end: "bottom center",
                        scrub: false,
                        once: true,
                    },
                    y: "100%",
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 0.4,
                });
            }
            // 푸터 설명
            var footerDescription = footer.querySelector(".footer-description");
            if (footerDescription) {
                gsap.from(footerDescription, {
                    scrollTrigger: {
                        trigger: footer,
                        start: "center-=100px center",
                        end: "center+=100px center",
                        scrub: false,
                        once: true,
                    },
                    y: 100,
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 0.8,
                });
            }
            // 푸터 연락처
            var footerContact = footer.querySelector(".footer-contact");
            if (footerContact) {
                gsap.from(footerContact, {
                    scrollTrigger: {
                        trigger: footer,
                        start: "top center",
                        end: "bottom center",
                        scrub: false,
                        once: true,
                    },
                    y: "100%",
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 0.4,
                });
            }
            // 푸터 링크
            var footerLinks = footer.querySelectorAll(".footer-links li .link");
            if (footerLinks.length > 0) {
                footerLinks.forEach((item, index) => {
                    gsap.from(item, {
                        scrollTrigger: {
                            trigger: footer,
                            start: "center center",
                            end: "center-=100px center",
                            scrub: false,
                            once: true,
                        },
                        y: 60,
                        opacity: 0,
                        ease: "sine.out",
                        duration: 0.2,
                        delay: 0.2 * index,
                    });
                });
            }
            // 푸터 소셜 링크
            var socialLinks = footer.querySelectorAll(".social-links li .link");
            if (socialLinks.length > 0) {
                socialLinks.forEach((item, index) => {
                    gsap.from(item, {
                        scrollTrigger: {
                            trigger: footer,
                            start: "center center",
                            end: "center-=100px center",
                            scrub: false,
                            once: true,
                        },
                        y: 60,
                        opacity: 0,
                        ease: "sine.out",
                        duration: 0.2,
                        delay: 0.2 * index,
                    });
                });
            }
        });
    }

    // GSAP 작업 목록 스크롤 트리거
    gsap.registerPlugin(ScrollTrigger);
    var works = document.querySelectorAll(".works");
    var windowWidth =
        window.innerWidth ||
        document.documentElement.clientWidth ||
        document.body.clientWidth;
    gsap.from(works, {
        y: 300,
        opacity: 0,
        ease: "sine.out",
        duration: 0.5,
        delay: 1,
    });
    works.forEach((works) => {
        if (works) {
            var workitem = works.querySelectorAll(".work-item");
            workitem.forEach((element) => {
                var workSerial = element.querySelector(".work-item-serial");
                var workTitle = element.querySelector(".work-item-title");
                var workMedia = element.querySelector(".work-item-media");
                var workMediaImage = element.querySelector(
                    ".work-item-media .image"
                );
                var workMediaVideo = element.querySelector(
                    ".work-item-media .video"
                );
                var workDescription = element.querySelector(
                    ".work-item-description"
                );
                if (workSerial) {
                    gsap.from(workSerial, {
                        y: "75%",
                        duration: 2,
                        scrollTrigger: {
                            trigger: element,
                            scrub: 1,
                        },
                    });
                }
                if (workTitle && windowWidth > 480) {
                    gsap.from(workTitle, {
                        opacity: 0.2,
                        duration: 1,
                        scrollTrigger: {
                            trigger: element,
                            scrub: 1,
                            start: "top top+=80%",
                            end: "bottom center",
                        },
                    });
                }
                if (workMediaImage && windowWidth > 480) {
                    gsap.to(workMediaImage, {
                        scale: 1.2,
                        duration: 2,
                        scrollTrigger: {
                            trigger: element,
                            scrub: 1,
                        },
                    });
                }
                if (workMediaVideo && windowWidth > 480) {
                    gsap.to(workMediaVideo, {
                        scale: 1.2,
                        duration: 2,
                        scrollTrigger: {
                            trigger: element,
                            scrub: 1,
                        },
                    });
                }
                if (workDescription && windowWidth > 480) {
                    gsap.from(workDescription, {
                        y: 200,
                        opacity: -1,
                        duration: 2,
                        scrollTrigger: {
                            trigger: workMedia,
                            scrub: 1,
                            start: "top+=100 center",
                            end: "bottom+=300 bottom",
                        },
                    });
                }
            });
        }
    });

    // GSAP 프로젝트 상세 스크롤 트리거
    if (projectDetailsContainer && windowWidth > 480) {
        gsap.registerPlugin(ScrollTrigger);
        // 프로젝트 상세 소개
        var aboutus = projectDetailsContainer.querySelector(".project-aboutus");
        if (aboutus) {
            var aboutusContents = aboutus.querySelectorAll(".content");
            aboutusContents.forEach((aboutusContent, index) => {
                gsap.from(aboutusContent, {
                    scrollTrigger: {
                        trigger: aboutus,
                        start: "top center",
                        end: "bottom center",
                        scrub: false,
                        once: true,
                    },
                    y: "100%",
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 0.4 * index,
                });
            });
        }
        // 프로젝트 상세 미디어 블록
        var projectMediaBlock = projectDetailsContainer.querySelectorAll(
            ".project-media-block"
        );
        if (projectMediaBlock) {
            projectMediaBlock.forEach((item, index) => {
                var video = item.querySelector(".video");
                var image = item.querySelector(".image");
                if (video) {
                    gsap.from(video, {
                        scrollTrigger: {
                            trigger: item,
                            start: "top center",
                            end: "bottom center",
                            scrub: false,
                            once: true,
                        },
                        y: 100,
                        opacity: 0,
                        ease: "power4.out",
                        duration: 1,
                        delay: 0.4,
                    });
                }
                if (image) {
                    gsap.from(image, {
                        scrollTrigger: {
                            trigger: item,
                            start: "top center",
                            end: "bottom center",
                            scrub: false,
                            once: true,
                        },
                        y: 100,
                        opacity: 0,
                        ease: "power4.out",
                        duration: 1,
                        delay: 0.4,
                    });
                }
            });
        }

        // 프로젝트 상세 텍스트 블록
        var projectTextBlock = projectDetailsContainer.querySelectorAll(
            ".project-text-block"
        );
        if (projectTextBlock) {
            projectTextBlock.forEach((item, index) => {
                var textBlockContent = item.querySelector(
                    ".text-block-content"
                );
                gsap.from(textBlockContent, {
                    scrollTrigger: {
                        trigger: item,
                        start: "top center",
                        end: "bottom center",
                        scrub: false,
                        once: true,
                    },
                    y: 100,
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 0.4,
                });
            });
        }
        // 프로젝트 상세 미디어 그리드 블록
        var projectMediaGridBlock = projectDetailsContainer.querySelectorAll(
            ".project-media-grid-block"
        );
        if (projectMediaGridBlock) {
            projectMediaGridBlock.forEach((item, index) => {
                var mediaGridItem = item.querySelectorAll(
                    ".media-grid-item > div"
                );
                gsap.from(mediaGridItem, {
                    scrollTrigger: {
                        trigger: mediaGridItem,
                        start: "top center",
                        end: "bottom center",
                        scrub: false,
                        once: true,
                    },
                    y: 100,
                    opacity: 0,
                    ease: "power4.out",
                    duration: 1,
                    delay: 0.4,
                    stagger: 0.2,
                });
            });
        }
    }

    // GSAP 연락처 제목
    if (contactTitle) {
        contactTitle.forEach((element) => {
            gsap.from(element, {
                y: 80,
                opacity: 0,
                ease: "power3.out",
                duration: 1,
                delay: 0.2,
            });
        });
    }

    // GSAP 연락처 정보
    if (contactInformations) {
        contactInformations.forEach((element) => {
            gsap.from(element, {
                y: 80,
                opacity: 0,
                ease: "power3.out",
                duration: 1,
                delay: 0.8,
            });
        });
    }

    // GSAP 연락처 폼
    if (contactForm) {
        gsap.from(contactForm, {
            y: 80,
            opacity: 0,
            ease: "power3.out",
            duration: 1,
            delay: 1.2,
        });
    }

    // GSAP 슬라이드 업 효과
    if (revealSlideUpItems.length > 0) {
        gsap.registerPlugin(ScrollTrigger);
        revealSlideUpItems.forEach((revealSlideUpItem) => {
            gsap.from(revealSlideUpItem, {
                scrollTrigger: {
                    trigger: revealSlideUpItem,
                    start: "top top+=90%",
                    end: "bottom center",
                    scrub: false,
                    once: true,
                },
                y: 100,
                opacity: 0,
                ease: "power4.out",
                duration: 1,
                delay: 1,
                skewY: 5,
                stagger: {
                    amount: 1,
                },
            });
        });
    }

    // GSAP 페이드 효과
    if (revealFadeItems.length > 0) {
        gsap.registerPlugin(ScrollTrigger);
        revealFadeItems.forEach((revealFadeItem) => {
            gsap.from(revealFadeItem, {
                scrollTrigger: {
                    trigger: revealFadeItem,
                    start: "top top+=90%",
                    end: "bottom center",
                    scrub: false,
                    once: true,
                },
                opacity: 0,
                ease: "power4.out",
                duration: 1,
                delay: 1,
                stagger: {
                    amount: 1,
                },
            });
        });
    }

    // 플로팅 입력 라벨
    var floatingFields = document.querySelectorAll(".floating-field");
    if (floatingFields) {
        floatingFields.forEach((item) => {
            var itemInput = item.querySelector(".float-input");
            var itemLabel = item.querySelector(".float-label");
            var value = itemInput.value;
            itemInput.addEventListener("focus", function () {
                gsap.to(itemLabel, {
                    y: -20,
                    scale: 0.5,
                    color: "#000000",
                    ease: "power4.out",
                    duration: 0.3,
                });
            });
            itemInput.addEventListener("blur", function () {
                if (!value) {
                    gsap.to(itemLabel, {
                        y: 0,
                        scale: 1,
                        color: "#E0E0E0",
                        ease: "power4.out",
                        duration: 0.3,
                    });
                }
            });
            itemInput.addEventListener("keyup", function () {
                value = itemInput.value;
            });
        });
    }

    // 스와이퍼 초기화
    const speed = 1000;

    // 첫 번째 제목 스와이퍼 인스턴스
    const projectTitleOneSwiper = new Swiper(".projects-title-1", {
        direction: "vertical",
        effect: "slide",
        autoHeight: true,
        loop: false,
        allowTouchMove: false,
    });

    // 기본 스와이퍼 인스턴스
    const projectsPrimarySwiper = new Swiper(".projects-primary", {
        slidesPerView: 1,
        spaceBetween: 1,
        speed: speed,
        loop: false,
        longSwipesRatio: 0.01,
        followFinger: false,
        grabCursor: true,
        watchSlidesProgress: true,
        parallax: true,
        lazy: {
            loadPrevNext: true,
        },
        navigation: {
            nextEl: ".swiper-button-next",
            prevEl: ".swiper-button-prev",
        },
    });

    // 보조 A 스와이퍼 인스턴스
    const projectsSecondaryOneSwiper = new Swiper(".projects-secondary-1", {
        slidesPerView: 1,
        spaceBetween: 1,
        speed: speed,
        loop: false,
        longSwipesRatio: 0.01,
        followFinger: false,
        grabCursor: true,
        watchSlidesProgress: true,
        parallax: true,
        allowTouchMove: true,
        lazy: {
            loadPrevNext: true,
        },
    });

    // 보조 B 스와이퍼 인스턴스
    const projectsSecondaryTwoSwiper = new Swiper(".projects-secondary-2", {
        slidesPerView: 1,
        spaceBetween: 1,
        speed: speed,
        loop: false,
        longSwipesRatio: 0.01,
        followFinger: false,
        grabCursor: true,
        watchSlidesProgress: true,
        parallax: true,
        allowTouchMove: true,
        lazy: {
            loadPrevNext: true,
        },
    });

    // 두 번째 제목 스와이퍼 인스턴스
    const projectTitleTwoSwiper = new Swiper(".projects-title-2", {
        direction: "vertical",
        effect: "slide",
        autoHeight: true,
        loop: false,
        allowTouchMove: false,
    });

    // 컨트롤러 관계 설정
    projectsPrimarySwiper.controller.control = [
        projectTitleOneSwiper,
        projectTitleTwoSwiper,
        projectsSecondaryOneSwiper,
        projectsSecondaryTwoSwiper,
    ];
    projectsSecondaryOneSwiper.controller.control = [projectsPrimarySwiper];
    projectsSecondaryTwoSwiper.controller.control = [projectsPrimarySwiper];

    // 서비스 토글
    if (serviceContainer) {
        var services = document.querySelectorAll(".service-item");
        services.forEach((service) => {
            service.addEventListener("mouseover", function () {
                service.classList.add("active");
            });
            service.addEventListener("mouseout", function () {
                service.classList.remove("active");
            });
        });
    }

    // 클라이언트 스와이퍼 인스턴스
    function clientSwiperInitialzation() {
        if (clientListSwiper) {
            if (windowWidth < 768) {
                const clientSwiper = new Swiper(clientListSwiper, {
                    slidesPerView: 1,
                    spaceBetween: 0,
                    navigation: {
                        nextEl: ".swiper-button-next.client-swiper-button",
                        prevEl: ".swiper-button-prev.client-swiper-button",
                    },
                    effect: "slide",
                    loop: true,
                    allowTouchMove: true,
                });
            }
        }
    }
    clientSwiperInitialzation();
    //window.addEventListener("load", clientSwiperInitialzation);
    window.addEventListener("resize", clientSwiperInitialzation);
    //window.addEventListener("scroll", clientSwiperInitialzation);

    // 미디어 그리드 3inC 스와이퍼 인스턴스
    function projectMediaGridBlock3inCInit() {
        if (projectMediaGridBlock3inC && windowWidth < 480) {
            projectMediaGridBlock3inC.forEach((element) => {
                var swiper3inC = new Swiper(element, {
                    slidesPerView: "auto",
                    spaceBetween: 0,
                });
            });
        }
    }
    projectMediaGridBlock3inCInit();
    //window.addEventListener("load", projectMediaGridBlock3inCInit);
    window.addEventListener("resize", projectMediaGridBlock3inCInit);
    //window.addEventListener("scroll", projectMediaGridBlock3inCInit);

    // 미디어 그리드 2inC 스와이퍼 인스턴스
    function projectMediaGridBlock2inCInit() {
        if (projectMediaGridBlock2inC && windowWidth < 480) {
            projectMediaGridBlock2inC.forEach((element) => {
                var swiper2inC = new Swiper(element, {
                    slidesPerView: "auto",
                    spaceBetween: 0,
                });
            });
        }
    }
    projectMediaGridBlock2inCInit();
    //window.addEventListener("load", projectMediaGridBlock2inCInit);
    window.addEventListener("resize", projectMediaGridBlock2inCInit);
    //window.addEventListener("scroll", projectMediaGridBlock2inCInit);

    // 전면 및 후면 레이어 섹션 높이 동일화
    logDivHeight(".layer-front .section-banner", ".layer-back .section-banner");
    logDivHeight(".layer-front .section-about", ".layer-back .section-about");
    logDivHeight(
        ".layer-front .section-services",
        ".layer-back .section-services"
    );
    logDivHeight(
        ".layer-front .section-clients",
        ".layer-back .section-clients"
    );
    logDivHeight(".layer-front footer", ".layer-back footer");
    logDivHeight(
        ".layer-front .section-project-details",
        ".layer-back .section-project-details"
    );
    logDivHeight(
        ".layer-front .section-contact",
        ".layer-back .section-contact"
    );
    logDivHeight(".layer-front .section-works", ".layer-back .section-works");
    logDivHeight(
        ".layer-front .section-services-banner",
        ".layer-back .section-services-banner"
    );
    logDivHeight(
        ".layer-front .section-services-teams",
        ".layer-back .section-services-teams"
    );
    logDivHeight(
        ".layer-front .section-services-collaborate",
        ".layer-back .section-services-collaborate"
    );
    logDivHeight(
        ".layer-front .section-services-why-us",
        ".layer-back .section-services-why-us"
    );
    logDivHeight(
        ".layer-front .section-services-works",
        ".layer-back .section-services-works"
    );
}

function canUseReinoLibraries() {
    return (
        typeof window !== "undefined" &&
        typeof window.gsap !== "undefined" &&
        typeof window.ScrollTrigger !== "undefined" &&
        typeof window.SmoothScroll !== "undefined" &&
        typeof window.bodymovin !== "undefined" &&
        typeof window.Swiper !== "undefined"
    );
}

function runReinoInit(attempt) {
    var currentAttempt = typeof attempt === "number" ? attempt : 0;
    if (!canUseReinoLibraries()) {
        if (currentAttempt < 50) {
            setTimeout(function () {
                runReinoInit(currentAttempt + 1);
            }, 100);
        } else {
            console.warn(
                "Reino animations skipped: required libraries not ready"
            );
        }
        return;
    }
    initializeReinoPage();
}

if (document.readyState === "loading") {
    document.addEventListener("DOMContentLoaded", function () {
        runReinoInit(0);
    });
} else {
    runReinoInit(0);
}

// React에서 호출할 수 있도록 전역에 노출
window.initializeReinoPage = initializeReinoPage;

// 로드 시 전면 및 후면 레이어 섹션 높이 동일화 함수 트리거
window.addEventListener("load", function () {
    resizeBodyHeight();
    logDivHeight(".layer-front .section-banner", ".layer-back .section-banner");
    logDivHeight(".layer-front .section-about", ".layer-back .section-about");
    logDivHeight(
        ".layer-front .section-services",
        ".layer-back .section-services"
    );
    logDivHeight(
        ".layer-front .section-clients",
        ".layer-back .section-clients"
    );
    logDivHeight(".layer-front footer", ".layer-back footer");
    logDivHeight(
        ".layer-front .section-project-details",
        ".layer-back .section-project-details"
    );
    logDivHeight(
        ".layer-front .section-contact",
        ".layer-back .section-contact"
    );
    logDivHeight(".layer-front .section-works", ".layer-back .section-works");
    logDivHeight(
        ".layer-front .section-services-banner",
        ".layer-back .section-services-banner"
    );
    logDivHeight(
        ".layer-front .section-services-teams",
        ".layer-back .section-services-teams"
    );
    logDivHeight(
        ".layer-front .section-services-collaborate",
        ".layer-back .section-services-collaborate"
    );
    logDivHeight(
        ".layer-front .section-services-why-us",
        ".layer-back .section-services-why-us"
    );
    logDivHeight(
        ".layer-front .section-services-works",
        ".layer-back .section-services-works"
    );
});
// 리사이즈 시 전면 및 후면 레이어 섹션 높이 동일화 함수 트리거
window.addEventListener("resize", function () {
    resizeBodyHeight();
    logDivHeight(".layer-front .section-banner", ".layer-back .section-banner");
    logDivHeight(".layer-front .section-about", ".layer-back .section-about");
    logDivHeight(
        ".layer-front .section-services",
        ".layer-back .section-services"
    );
    logDivHeight(
        ".layer-front .section-clients",
        ".layer-back .section-clients"
    );
    logDivHeight(".layer-front footer", ".layer-back footer");
    logDivHeight(
        ".layer-front .section-project-details",
        ".layer-back .section-project-details"
    );
    logDivHeight(
        ".layer-front .section-contact",
        ".layer-back .section-contact"
    );
    logDivHeight(".layer-front .section-works", ".layer-back .section-works");
    logDivHeight(
        ".layer-front .section-services-banner",
        ".layer-back .section-services-banner"
    );
    logDivHeight(
        ".layer-front .section-services-teams",
        ".layer-back .section-services-teams"
    );
    logDivHeight(
        ".layer-front .section-services-collaborate",
        ".layer-back .section-services-collaborate"
    );
    logDivHeight(
        ".layer-front .section-services-why-us",
        ".layer-back .section-services-why-us"
    );
    logDivHeight(
        ".layer-front .section-services-works",
        ".layer-back .section-services-works"
    );
});
// 스크롤 시 전면 및 후면 레이어 섹션 높이 동일화 함수 트리거 - 성능 최적화를 위해 제거됨
/*
window.addEventListener("scroll", function () {
    resizeBodyHeight();
    logDivHeight(".layer-front .section-banner", ".layer-back .section-banner");
    logDivHeight(".layer-front .section-about", ".layer-back .section-about");
    logDivHeight(
        ".layer-front .section-services",
        ".layer-back .section-services"
    );
    logDivHeight(
        ".layer-front .section-clients",
        ".layer-back .section-clients"
    );
    logDivHeight(".layer-front footer", ".layer-back footer");
    logDivHeight(
        ".layer-front .section-project-details",
        ".layer-back .section-project-details"
    );
    logDivHeight(
        ".layer-front .section-contact",
        ".layer-back .section-contact"
    );
    logDivHeight(".layer-front .section-works", ".layer-back .section-works");
    logDivHeight(
        ".layer-front .section-services-banner",
        ".layer-back .section-services-banner"
    );
    logDivHeight(
        ".layer-front .section-services-teams",
        ".layer-back .section-services-teams"
    );
    logDivHeight(
        ".layer-front .section-services-collaborate",
        ".layer-back .section-services-collaborate"
    );
    logDivHeight(
        ".layer-front .section-services-why-us",
        ".layer-back .section-services-why-us"
    );
    logDivHeight(
        ".layer-front .section-services-works",
        ".layer-back .section-services-works"
    );
});
*/

// 비디오 성능 최적화: 화면에 보일 때만 재생
document.addEventListener("DOMContentLoaded", function () {
    var videos = document.querySelectorAll("video");
    if ("IntersectionObserver" in window) {
        var videoObserver = new IntersectionObserver(function (
            entries,
            observer
        ) {
            entries.forEach(function (entry) {
                if (entry.isIntersecting) {
                    entry.target.play();
                } else {
                    entry.target.pause();
                }
            });
        });

        videos.forEach(function (video) {
            videoObserver.observe(video);
        });
    }
});
