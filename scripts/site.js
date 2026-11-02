// Progressive enhancement: content is readable without JavaScript.
document.body.classList.add('loaded');
document.getElementById('navbarToggler')?.addEventListener('click', event => {
  const link = event.target.closest('a');
  const menu = event.currentTarget;
  if (link && !link.matches('[data-bs-toggle]') && menu.classList.contains('show')) {
    window.bootstrap?.Collapse.getOrCreateInstance(menu, {toggle:false}).hide();
  }
});
