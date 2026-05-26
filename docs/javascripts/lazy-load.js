/* 图片懒加载 */
(function () {
  function initLazyLoad() {
    // 为所有图片添加 loading="lazy" 属性
    const images = document.querySelectorAll('.md-content img:not([loading])');
    images.forEach(img => {
      // 跳过 logo 和小图标
      if (img.closest('.md-logo') || img.closest('.md-header') || img.alt === 'logo') {
        return;
      }
      img.setAttribute('loading', 'lazy');
      img.setAttribute('decoding', 'async');
    });
  }

  /* MkDocs Material SPA 模式使用 document$ observable */
  if (typeof document$ !== 'undefined') {
    document$.subscribe(initLazyLoad);
  } else {
    document.addEventListener('DOMContentLoaded', initLazyLoad);
  }
})();
