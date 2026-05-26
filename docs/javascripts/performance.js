/* 字体预加载优化 */
(function () {
  function preloadFonts() {
    // 预加载 JetBrains Mono 字体（代码块使用）
    const fontLinks = [
      {
        href: 'https://cdn.jsdelivr.net/npm/jetbrains-mono@1.0.6/css/jetbrains-mono.min.css',
        rel: 'preload',
        as: 'style'
      }
    ];

    fontLinks.forEach(link => {
      const existingPreload = document.querySelector(`link[rel="preload"][href="${link.href}"]`);
      if (!existingPreload) {
        const preloadLink = document.createElement('link');
        preloadLink.rel = link.rel;
        preloadLink.href = link.href;
        preloadLink.as = link.as;
        preloadLink.crossOrigin = 'anonymous';
        document.head.appendChild(preloadLink);
      }
    });
  }

  // DNS 预解析
  function preconnectDomains() {
    const domains = [
      'https://cdn.jsdelivr.net',
      'https://cdnjs.cloudflare.com',
      'https://giscus.app'
    ];

    domains.forEach(domain => {
      const existingPreconnect = document.querySelector(`link[rel="preconnect"][href="${domain}"]`);
      if (!existingPreconnect) {
        const preconnectLink = document.createElement('link');
        preconnectLink.rel = 'preconnect';
        preconnectLink.href = domain;
        preconnectLink.crossOrigin = 'anonymous';
        document.head.appendChild(preconnectLink);
      }
    });
  }

  // 在页面加载完成后执行
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', () => {
      preloadFonts();
      preconnectDomains();
    });
  } else {
    preloadFonts();
    preconnectDomains();
  }
})();
