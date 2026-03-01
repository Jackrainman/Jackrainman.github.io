/* IDE theme: override search placeholder text */
(function () {
  function setPlaceholder() {
    var input = document.querySelector('.md-search__input');
    if (input) input.placeholder = 'Ctrl+K 搜索...';
  }

  /* MkDocs Material SPA mode uses document$ observable */
  if (typeof document$ !== 'undefined') {
    document$.subscribe(setPlaceholder);
  } else {
    document.addEventListener('DOMContentLoaded', setPlaceholder);
  }
})();
