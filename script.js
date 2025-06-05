/**
 * Automation Engineering Hub - Main JavaScript
 * Handles interactivity, animations, and responsive behavior
 */

document.addEventListener('DOMContentLoaded', function() {
    // Initialize all interactive components
    initNavigation();
    initScrollAnimations();
    initExpandableCards();
    initTabs();
    initAccordions();
    initLanguageSelector();
    initRobotVisualization();
    initTimelineFilters();
    initNewsletterForm();
    
    // Add circuit animation to hero section
    animateCircuits();
});

/**
 * Mobile Navigation Toggle
 */
function initNavigation() {
    const menuToggle = document.querySelector('.menu-toggle');
    const navLinks = document.querySelector('.nav-links');
    
    if (menuToggle && navLinks) {
        menuToggle.addEventListener('click', function() {
            navLinks.classList.toggle('active');
            
            // Toggle hamburger to X animation
            const bars = menuToggle.querySelectorAll('.bar');
            bars.forEach(bar => bar.classList.toggle('active'));
            
            if (navLinks.classList.contains('active')) {
                menuToggle.setAttribute('aria-expanded', 'true');
            } else {
                menuToggle.setAttribute('aria-expanded', 'false');
            }
        });
        
        // Close menu when clicking on a link
        const links = navLinks.querySelectorAll('a');
        links.forEach(link => {
            link.addEventListener('click', function() {
                navLinks.classList.remove('active');
                menuToggle.setAttribute('aria-expanded', 'false');
                
                const bars = menuToggle.querySelectorAll('.bar');
                bars.forEach(bar => bar.classList.remove('active'));
            });
        });
    }
    
    // Header scroll effect
    const header = document.querySelector('.header');
    if (header) {
        window.addEventListener('scroll', function() {
            if (window.scrollY > 50) {
                header.classList.add('scrolled');
            } else {
                header.classList.remove('scrolled');
            }
        });
    }
}

/**
 * Scroll Animations
 */
function initScrollAnimations() {
    // Fade in elements as they enter the viewport
    const fadeElements = document.querySelectorAll('.fade-in');
    
    const fadeInObserver = new IntersectionObserver((entries) => {
        entries.forEach(entry => {
            if (entry.isIntersecting) {
                entry.target.classList.add('visible');
                fadeInObserver.unobserve(entry.target);
            }
        });
    }, {
        threshold: 0.1,
        rootMargin: '0px 0px -50px 0px'
    });
    
    fadeElements.forEach(element => {
        fadeInObserver.observe(element);
    });
}

/**
 * Expandable Cards
 */
function initExpandableCards() {
    const expandButtons = document.querySelectorAll('.expand-btn');
    
    expandButtons.forEach(button => {
        button.addEventListener('click', function() {
            const targetId = this.getAttribute('data-target');
            const expandedContent = document.getElementById(targetId);
            
            if (expandedContent) {
                if (expandedContent.style.display === 'block') {
                    expandedContent.style.display = 'none';
                    this.textContent = 'Learn More';
                } else {
                    expandedContent.style.display = 'block';
                    this.textContent = 'Show Less';
                }
            }
        });
    });
}

/**
 * Tabs Functionality
 */
function initTabs() {
    const tabButtons = document.querySelectorAll('.tab-btn');
    
    tabButtons.forEach(button => {
        button.addEventListener('click', function() {
            const tabId = this.getAttribute('data-tab');
            const tabContent = document.getElementById(tabId);
            const tabContainer = this.closest('.tabs-container');
            
            if (tabContainer && tabContent) {
                // Remove active class from all buttons and content
                tabContainer.querySelectorAll('.tab-btn').forEach(btn => {
                    btn.classList.remove('active');
                });
                
                tabContainer.querySelectorAll('.tab-content').forEach(content => {
                    content.classList.remove('active');
                });
                
                // Add active class to clicked button and corresponding content
                this.classList.add('active');
                tabContent.classList.add('active');
            }
        });
    });
}

/**
 * Accordions Functionality
 */
function initAccordions() {
    const accordionHeaders = document.querySelectorAll('.accordion-header');
    
    accordionHeaders.forEach(header => {
        header.addEventListener('click', function() {
            const accordion = this.parentElement;
            const isActive = accordion.classList.contains('active');
            
            // Close all accordions
            document.querySelectorAll('.accordion').forEach(acc => {
                acc.classList.remove('active');
            });
            
            // Toggle current accordion
            if (!isActive) {
                accordion.classList.add('active');
            }
        });
    });
}

/**
 * Language Selector for Programming Section
 */
function initLanguageSelector() {
    const languageButtons = document.querySelectorAll('.language-btn');
    
    languageButtons.forEach(button => {
        button.addEventListener('click', function() {
            const language = this.getAttribute('data-language');
            const languageContent = document.getElementById(`${language}-content`);
            
            // Remove active class from all buttons and content
            document.querySelectorAll('.language-btn').forEach(btn => {
                btn.classList.remove('active');
            });
            
            document.querySelectorAll('.language-content').forEach(content => {
                content.classList.remove('active');
            });
            
            // Add active class to clicked button and corresponding content
            this.classList.add('active');
            if (languageContent) {
                languageContent.classList.add('active');
            }
        });
    });
}

/**
 * Interactive Robot Visualization
 */
function initRobotVisualization() {
    const indicators = document.querySelectorAll('.indicator');
    
    indicators.forEach(indicator => {
        indicator.addEventListener('click', function() {
            const component = this.getAttribute('data-component');
            const componentInfo = document.getElementById(component);
            
            // Hide all component info
            document.querySelectorAll('.component-info').forEach(info => {
                info.classList.remove('active');
            });
            
            // Show selected component info
            if (componentInfo) {
                componentInfo.classList.add('active');
            }
            
            // Highlight selected indicator
            indicators.forEach(ind => {
                ind.classList.remove('active');
            });
            this.classList.add('active');
        });
    });
    
    // Create simple 3D robot model with Three.js if available
    if (typeof THREE !== 'undefined') {
        createRobotModel();
    } else {
        // Fallback to 2D representation
        create2DRobotVisualization();
    }
}

/**
 * Create a simple 2D robot visualization
 */
function create2DRobotVisualization() {
    const robotModel = document.querySelector('.robot-model');
    
    if (robotModel) {
        // Create a simple SVG robot representation
        const svgNS = "http://www.w3.org/2000/svg";
        const svg = document.createElementNS(svgNS, "svg");
        svg.setAttribute("width", "100%");
        svg.setAttribute("height", "100%");
        svg.setAttribute("viewBox", "0 0 200 300");
        
        // Robot head
        const head = document.createElementNS(svgNS, "rect");
        head.setAttribute("x", "70");
        head.setAttribute("y", "30");
        head.setAttribute("width", "60");
        head.setAttribute("height", "60");
        head.setAttribute("rx", "10");
        head.setAttribute("fill", "#1E88E5");
        
        // Robot eyes
        const leftEye = document.createElementNS(svgNS, "circle");
        leftEye.setAttribute("cx", "85");
        leftEye.setAttribute("cy", "50");
        leftEye.setAttribute("r", "8");
        leftEye.setAttribute("fill", "#FFFFFF");
        
        const rightEye = document.createElementNS(svgNS, "circle");
        rightEye.setAttribute("cx", "115");
        rightEye.setAttribute("cy", "50");
        rightEye.setAttribute("r", "8");
        rightEye.setAttribute("fill", "#FFFFFF");
        
        // Robot body
        const body = document.createElementNS(svgNS, "rect");
        body.setAttribute("x", "60");
        body.setAttribute("y", "100");
        body.setAttribute("width", "80");
        body.setAttribute("height", "100");
        body.setAttribute("fill", "#26A69A");
        
        // Robot arms
        const leftArm = document.createElementNS(svgNS, "rect");
        leftArm.setAttribute("x", "30");
        leftArm.setAttribute("y", "110");
        leftArm.setAttribute("width", "30");
        leftArm.setAttribute("height", "15");
        leftArm.setAttribute("fill", "#1E88E5");
        
        const rightArm = document.createElementNS(svgNS, "rect");
        rightArm.setAttribute("x", "140");
        rightArm.setAttribute("y", "110");
        rightArm.setAttribute("width", "30");
        rightArm.setAttribute("height", "15");
        rightArm.setAttribute("fill", "#1E88E5");
        
        // Robot legs
        const leftLeg = document.createElementNS(svgNS, "rect");
        leftLeg.setAttribute("x", "70");
        leftLeg.setAttribute("y", "200");
        leftLeg.setAttribute("width", "20");
        leftLeg.setAttribute("height", "70");
        leftLeg.setAttribute("fill", "#1E88E5");
        
        const rightLeg = document.createElementNS(svgNS, "rect");
        rightLeg.setAttribute("x", "110");
        rightLeg.setAttribute("y", "200");
        rightLeg.setAttribute("width", "20");
        rightLeg.setAttribute("height", "70");
        rightLeg.setAttribute("fill", "#1E88E5");
        
        // Add all elements to SVG
        svg.appendChild(head);
        svg.appendChild(leftEye);
        svg.appendChild(rightEye);
        svg.appendChild(body);
        svg.appendChild(leftArm);
        svg.appendChild(rightArm);
        svg.appendChild(leftLeg);
        svg.appendChild(rightLeg);
        
        // Add SVG to robot model container
        robotModel.appendChild(svg);
        
        // Add simple animation
        let angle = 0;
        setInterval(() => {
            angle += 0.05;
            const y = Math.sin(angle) * 5;
            head.setAttribute("transform", `translate(0, ${y})`);
            leftArm.setAttribute("transform", `rotate(${Math.sin(angle) * 10}, 30, 117.5)`);
            rightArm.setAttribute("transform", `rotate(${-Math.sin(angle) * 10}, 170, 117.5)`);
        }, 50);
    }
}

/**
 * Timeline Filters
 */
function initTimelineFilters() {
    const filterButtons = document.querySelectorAll('.filter-btn');
    const timelineItems = document.querySelectorAll('.timeline-item');
    
    filterButtons.forEach(button => {
        button.addEventListener('click', function() {
            const filter = this.getAttribute('data-filter');
            
            // Remove active class from all buttons
            filterButtons.forEach(btn => {
                btn.classList.remove('active');
            });
            
            // Add active class to clicked button
            this.classList.add('active');
            
            // Filter timeline items
            timelineItems.forEach(item => {
                if (filter === 'all' || item.getAttribute('data-category') === filter) {
                    item.style.display = 'flex';
                } else {
                    item.style.display = 'none';
                }
            });
        });
    });
}

/**
 * Newsletter Form Submission
 */
function initNewsletterForm() {
    const newsletterForm = document.querySelector('.signup-form');
    
    if (newsletterForm) {
        newsletterForm.addEventListener('submit', function(e) {
            e.preventDefault();
            
            const emailInput = this.querySelector('input[type="email"]');
            const email = emailInput.value.trim();
            
            if (email) {
                // In a real application, this would send the email to a server
                // For now, just show a success message
                emailInput.value = '';
                
                const successMessage = document.createElement('p');
                successMessage.textContent = 'Thank you for subscribing!';
                successMessage.style.color = 'var(--success-color)';
                successMessage.style.marginTop = 'var(--spacing-md)';
                
                // Remove any existing messages
                const existingMessage = newsletterForm.nextElementSibling;
                if (existingMessage && existingMessage.tagName === 'P') {
                    existingMessage.remove();
                }
                
                newsletterForm.parentNode.insertBefore(successMessage, newsletterForm.nextSibling);
                
                // Hide message after 3 seconds
                setTimeout(() => {
                    successMessage.style.opacity = '0';
                    successMessage.style.transition = 'opacity 0.5s ease';
                    
                    setTimeout(() => {
                        successMessage.remove();
                    }, 500);
                }, 3000);
            }
        });
    }
}

/**
 * Circuit Animation in Hero Section
 */
function animateCircuits() {
    const circuitAnimation = document.querySelector('.circuit-animation');
    
    if (circuitAnimation) {
        // Create animated circuit lines
        for (let i = 0; i < 20; i++) {
            const line = document.createElement('div');
            line.classList.add('circuit-line');
            
            // Random positioning and sizing
            line.style.position = 'absolute';
            line.style.width = `${Math.random() * 100 + 50}px`;
            line.style.height = '2px';
            line.style.backgroundColor = 'rgba(255, 255, 255, 0.2)';
            line.style.top = `${Math.random() * 100}%`;
            line.style.left = `${Math.random() * 100}%`;
            line.style.transform = `rotate(${Math.random() * 360}deg)`;
            
            // Animation
            line.style.animation = `pulse ${Math.random() * 3 + 2}s infinite alternate`;
            
            circuitAnimation.appendChild(line);
            
            // Create circuit nodes at the ends of some lines
            if (i % 3 === 0) {
                const node = document.createElement('div');
                node.classList.add('circuit-node');
                node.style.position = 'absolute';
                node.style.width = '6px';
                node.style.height = '6px';
                node.style.borderRadius = '50%';
                node.style.backgroundColor = 'rgba(255, 255, 255, 0.4)';
                node.style.top = line.style.top;
                node.style.left = line.style.left;
                
                // Animation
                node.style.animation = `glow ${Math.random() * 2 + 1}s infinite alternate`;
                
                circuitAnimation.appendChild(node);
            }
        }
    }
    
    // Add keyframes for animations
    const style = document.createElement('style');
    style.textContent = `
        @keyframes pulse {
            0% { opacity: 0.2; }
            100% { opacity: 0.8; }
        }
        
        @keyframes glow {
            0% { box-shadow: 0 0 2px rgba(30, 136, 229, 0.5); }
            100% { box-shadow: 0 0 8px rgba(30, 136, 229, 0.8); }
        }
    `;
    document.head.appendChild(style);
}

/**
 * Smooth scrolling for anchor links
 */
document.querySelectorAll('a[href^="#"]').forEach(anchor => {
    anchor.addEventListener('click', function(e) {
        e.preventDefault();
        
        const targetId = this.getAttribute('href');
        const targetElement = document.querySelector(targetId);
        
        if (targetElement) {
            // Get header height for offset
            const headerHeight = document.querySelector('.header').offsetHeight;
            const targetPosition = targetElement.getBoundingClientRect().top + window.pageYOffset - headerHeight;
            
            window.scrollTo({
                top: targetPosition,
                behavior: 'smooth'
            });
        }
    });
});

/**
 * Code syntax highlighting
 * This is a simple implementation. In a production environment,
 * you might want to use a library like Prism.js or Highlight.js
 */
function highlightCode() {
    const codeBlocks = document.querySelectorAll('pre code');
    
    codeBlocks.forEach(block => {
        // Simple syntax highlighting for Python
        if (block.classList.contains('language-python')) {
            let html = block.innerHTML;
            
            // Keywords
            const pythonKeywords = ['def', 'class', 'import', 'from', 'if', 'else', 'elif', 'for', 'while', 'try', 'except', 'return', 'and', 'or', 'not', 'in', 'is', 'True', 'False', 'None'];
            pythonKeywords.forEach(keyword => {
                const regex = new RegExp(`\\b${keyword}\\b`, 'g');
                html = html.replace(regex, `<span class="keyword">${keyword}</span>`);
            });
            
            // Strings
            html = html.replace(/(["'])(?:(?=(\\?))\2.)*?\1/g, '<span class="string">$&</span>');
            
            // Comments
            html = html.replace(/#.*$/gm, '<span class="comment">$&</span>');
            
            block.innerHTML = html;
        }
        
        // Simple syntax highlighting for C
        if (block.classList.contains('language-c')) {
            let html = block.innerHTML;
            
            // Keywords
            const cKeywords = ['int', 'float', 'char', 'void', 'double', 'return', 'if', 'else', 'for', 'while', 'do', 'switch', 'case', 'break', 'continue', 'struct', 'typedef', 'const', 'static', 'extern', 'volatile', 'register', 'sizeof'];
            cKeywords.forEach(keyword => {
                const regex = new RegExp(`\\b${keyword}\\b`, 'g');
                html = html.replace(regex, `<span class="keyword">${keyword}</span>`);
            });
            
            // Preprocessor directives
            html = html.replace(/#\w+/g, '<span class="preprocessor">$&</span>');
            
            // Strings
            html = html.replace(/(["'])(?:(?=(\\?))\2.)*?\1/g, '<span class="string">$&</span>');
            
            // Comments
            html = html.replace(/\/\/.*$/gm, '<span class="comment">$&</span>');
            html = html.replace(/\/\*[\s\S]*?\*\//g, '<span class="comment">$&</span>');
            
            block.innerHTML = html;
        }
    });
    
    // Add CSS for syntax highlighting
    const style = document.createElement('style');
    style.textContent = `
        .keyword { color: #569CD6; }
        .string { color: #CE9178; }
        .comment { color: #6A9955; }
        .preprocessor { color: #C586C0; }
    `;
    document.head.appendChild(style);
}

// Call syntax highlighting after DOM is loaded
document.addEventListener('DOMContentLoaded', highlightCode);
