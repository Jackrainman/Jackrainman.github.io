/* Stats 数字滚动动画 */
(function () {
  function animateStats() {
    const stats = document.querySelectorAll('.landing-stat-value');
    if (stats.length === 0) return;

    // 使用 Intersection Observer 确保元素可见时才触发动画
    const observer = new IntersectionObserver((entries) => {
      entries.forEach(entry => {
        if (entry.isIntersecting) {
          animateNumber(entry.target);
          observer.unobserve(entry.target);
        }
      });
    }, { threshold: 0.5 });

    stats.forEach(stat => observer.observe(stat));
  }

  function animateNumber(element) {
    const target = element.textContent;
    // 跳过非数字内容（如 ∞）
    if (target === '∞') {
      element.style.opacity = '1';
      return;
    }

    const hasPlus = target.includes('+');
    const numericValue = parseInt(target);
    if (isNaN(numericValue)) {
      element.style.opacity = '1';
      return;
    }

    // 初始状态
    element.style.opacity = '0';
    element.style.transform = 'translateY(10px)';
    element.style.transition = 'opacity 0.3s ease, transform 0.3s ease';

    // 延迟开始动画
    setTimeout(() => {
      element.style.opacity = '1';
      element.style.transform = 'translateY(0)';

      // 数字滚动效果
      let current = 0;
      const duration = 1000; // 1秒
      const startTime = performance.now();

      function updateNumber(currentTime) {
        const elapsed = currentTime - startTime;
        const progress = Math.min(elapsed / duration, 1);

        // 使用 easeOutQuad 缓动函数
        const easedProgress = 1 - (1 - progress) * (1 - progress);
        current = Math.floor(easedProgress * numericValue);

        element.textContent = current + (hasPlus ? '+' : '');

        if (progress < 1) {
          requestAnimationFrame(updateNumber);
        } else {
          element.textContent = target; // 确保最终值准确
        }
      }

      requestAnimationFrame(updateNumber);
    }, 800); // Hero 动画完成后开始
  }

  /* MkDocs Material SPA 模式使用 document$ observable */
  if (typeof document$ !== 'undefined') {
    document$.subscribe(() => {
      // 等待 DOM 更新完成
      setTimeout(animateStats, 100);
    });
  } else {
    document.addEventListener('DOMContentLoaded', animateStats);
  }
})();
