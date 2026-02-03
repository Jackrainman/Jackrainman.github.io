// MathJax 配置 - 必须在加载 MathJax 库之前执行
window.MathJax = {
  tex: {
    inlineMath: [['\\(', '\\)'], ['$', '$']],
    displayMath: [['\\[', '\\]'], ['$$', '$$']],
    processEscapes: true,
    processEnvironments: true,
    packages: {'[+]': ['ams']}
  },
  options: {
    // 处理所有元素，不只是 arithmatex 类
    skipHtmlTags: ['script', 'noscript', 'style', 'textarea', 'pre'],
    ignoreHtmlClass: '',
    processHtmlClass: ''
  },
  startup: {
    pageReady: function() {
      console.log('[MathJax] Initializing...');
      return MathJax.typesetPromise().then(function() {
        console.log('[MathJax] Typeset complete');
      });
    }
  }
};

// MkDocs Material 页面切换时重新渲染
document$.subscribe(function() {
  if (typeof MathJax !== 'undefined' && MathJax.typesetPromise) {
    console.log('[MathJax] Re-typesetting after navigation...');
    MathJax.typesetPromise();
  }
});

// Pangu 自动空格
document$.subscribe(function() {
  if (typeof pangu !== 'undefined' && pangu.spacingPageBody) {
    setTimeout(function() { pangu.spacingPageBody(); }, 100);
  }
});
