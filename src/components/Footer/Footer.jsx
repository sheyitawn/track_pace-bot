import React from "react";
import "./footer.css";

export default function Footer() {
  return (
    <footer className="site-footer">
      <a className="brand" href="https://sheyitawn.dev" target="_blank" rel="noreferrer">
        <img src="/logo.png" alt="Sheyitan — website" className="brand-logo" />
        <span className="brand-text">sheyitawn.dev</span>
      </a>
      <span className="tagline">Track Pace-Bot Control Panel</span>
    </footer>
  );
}
