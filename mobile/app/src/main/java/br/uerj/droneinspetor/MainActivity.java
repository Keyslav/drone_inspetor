package br.uerj.droneinspetor;

import android.annotation.SuppressLint;
import android.app.Activity;
import android.app.AlertDialog;
import android.content.ActivityNotFoundException;
import android.content.Intent;
import android.graphics.Bitmap;
import android.graphics.Insets;
import android.graphics.Rect;
import android.graphics.Color;
import android.os.Bundle;
import android.os.Build;
import android.content.res.Configuration;
import android.os.SystemClock;
import android.speech.RecognizerIntent;
import android.text.InputType;
import android.view.WindowInsets;
import android.view.WindowInsetsController;
import android.view.WindowManager;
import android.view.View;
import android.webkit.CookieManager;
import android.webkit.WebResourceResponse;
import android.webkit.WebResourceRequest;
import android.webkit.WebSettings;
import android.webkit.WebView;
import android.webkit.WebViewClient;
import android.widget.FrameLayout;
import android.widget.EditText;

import android.widget.Toast;
import java.net.URI;
import java.io.BufferedReader;
import java.io.ByteArrayInputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.ArrayList;
import java.util.Locale;
import org.json.JSONObject;

/** Contêiner Android sem ponte JavaScript/nativa: todos os comandos passam pela API autenticada. */
public final class MainActivity extends Activity {
    private static final int SPEECH_REQUEST = 101;
    private static final String OFFLINE_ORIGIN = "https://appassets.androidplatform.net";
    private WebView web;
    private FrameLayout root;
    private volatile String gateway = "";
    // WebView invokes interception on its IO thread; publish a complete immutable allowlist.
    private volatile Set<String> bundledAssets = Collections.emptySet();
    private String speechOrigin;
    private String pendingTranscript;
    private long speechStarted;
    private long pageGeneration;
    private long speechGeneration;
    private boolean resumed;

    @SuppressLint("SetJavaScriptEnabled")
    @Override public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        root = new FrameLayout(this);
        root.setBackgroundColor(Color.rgb(11, 19, 33));
        web = new WebView(this);
        web.setOverScrollMode(View.OVER_SCROLL_NEVER);
        web.setHorizontalScrollBarEnabled(false);
        web.setVerticalScrollBarEnabled(false);
        root.addView(web, new FrameLayout.LayoutParams(-1, -1));
        setContentView(root);
        getWindow().setSoftInputMode(WindowManager.LayoutParams.SOFT_INPUT_ADJUST_RESIZE);
        configureWindowInsets();
        bundledAssets = readBundledAssets();

        WebSettings settings = web.getSettings();
        settings.setJavaScriptEnabled(true);
        settings.setDomStorageEnabled(false);
        settings.setAllowFileAccess(false);
        settings.setAllowContentAccess(false);
        settings.setGeolocationEnabled(false);
        settings.setSupportMultipleWindows(false);
        settings.setJavaScriptCanOpenWindowsAutomatically(false);
        // The dashboard only receives muted camera streams; autoplay needs no capture permission.
        settings.setMediaPlaybackRequiresUserGesture(false);
        settings.setMixedContentMode(WebSettings.MIXED_CONTENT_NEVER_ALLOW);
        settings.setCacheMode(WebSettings.LOAD_NO_CACHE);
        settings.setSafeBrowsingEnabled(true);
        CookieManager.getInstance().setAcceptCookie(false);
        CookieManager.getInstance().setAcceptThirdPartyCookies(web, false);
        web.setBackgroundColor(Color.rgb(11, 19, 33));
        web.setWebViewClient(new WebViewClient() {
            @Override public void onPageStarted(WebView view, String url, Bitmap favicon) {
                pageGeneration++;
                clearSpeech();
            }
            @Override public boolean shouldOverrideUrlLoading(WebView view, WebResourceRequest request) {
                String url = request.getUrl().toString();
                if (url.equals("drone-inspetor://station") || url.equals("drone-inspetor://speech")) {
                    // No generic JS bridge: only explicit, user-initiated actions from our own shell.
                    if (request.isForMainFrame() && request.hasGesture()
                            && sameOrigin(view.getUrl(), currentOrigin())) {
                        if (url.endsWith("station")) chooseStation(); else startSpeech();
                    }
                    return true;
                }
                // Only the embedded root page may navigate; API responses are never top-level HTML.
                return !isShellUrl(url);
            }
            @Override public WebResourceResponse shouldInterceptRequest(WebView view,
                    WebResourceRequest request) {
                String url = request.getUrl().toString();
                if (!sameOrigin(url, currentOrigin())) return null;
                String path = request.getUrl().getPath();
                if (path == null) return emptyResponse(404, "Not Found");
                if (path.equals("/api") || path.startsWith("/api/")) {
                    // Credentials stay in same-origin fetches. With no station, no API request leaves.
                    return gateway.isEmpty() ? emptyResponse(503, "Service Unavailable") : null;
                }
                if (!request.getMethod().equals("GET")) return emptyResponse(405, "Method Not Allowed");
                String encodedPath = request.getUrl().getEncodedPath();
                if (encodedPath == null || encodedPath.contains("%") || path.contains("..")
                        || path.contains("\\")) return emptyResponse(404, "Not Found");
                if (path.equals("/")) path = "/index.html";
                if (!bundledAssets.contains(path)) return emptyResponse(404, "Not Found");
                try {
                    InputStream content = getAssets().open("dashboard" + path);
                    return new WebResourceResponse(mimeType(path), path.endsWith(".png") ? null : "UTF-8", 200, "OK",
                        responseHeaders(), content);
                } catch (IOException error) {
                    return emptyResponse(404, "Not Found");
                }
            }
            // O comportamento padrão rejeita certificados TLS inválidos; nunca chamar proceed().
        });
        String savedGateway = getPreferences(MODE_PRIVATE).getString("gateway", "");
        try { gateway = savedGateway.isEmpty() ? "" : normalizeOrigin(savedGateway); }
        catch (IllegalArgumentException error) { gateway = ""; }
        // Always open the packaged dashboard, even when the user has not configured a station yet.
        openStation();
    }

    private String currentOrigin() {
        return gateway.isEmpty() ? OFFLINE_ORIGIN : gateway;
    }

    private boolean isShellUrl(String value) {
        if (!sameOrigin(value, currentOrigin())) return false;
        try {
            String path = new URI(value).getPath();
            return path == null || path.equals("/") || path.equals("/index.html");
        } catch (Exception error) { return false; }
    }

    private Set<String> readBundledAssets() {
        HashSet<String> paths = new HashSet<>();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(
                getAssets().open("dashboard-assets.txt"), StandardCharsets.UTF_8))) {
            String path;
            while ((path = reader.readLine()) != null) {
                if (path.startsWith("/") && !path.contains("..") && !path.contains("\\")) paths.add(path);
            }
        } catch (IOException error) {
            throw new IllegalStateException("Dashboard assets missing from APK", error);
        }
        return Collections.unmodifiableSet(paths);
    }

    private static String mimeType(String path) {
        if (path.endsWith(".html")) return "text/html";
        if (path.endsWith(".css")) return "text/css";
        if (path.endsWith(".js")) return "application/javascript";
        if (path.endsWith(".svg")) return "image/svg+xml";
        if (path.endsWith(".png")) return "image/png";
        return "text/plain";
    }

    private static Map<String, String> responseHeaders() {
        HashMap<String, String> headers = new HashMap<>();
        headers.put("Cache-Control", "no-store");
        headers.put("X-Content-Type-Options", "nosniff");
        headers.put("Content-Security-Policy", "default-src 'self'; script-src 'self'; "
            + "style-src 'self' 'unsafe-inline'; img-src 'self' data: blob: https://tile.openstreetmap.org https://*.tile.openstreetmap.org; "
            + "media-src 'self' blob:; connect-src 'self'; object-src 'none'; base-uri 'none'; "
            + "frame-ancestors 'none'; form-action 'none'");
        return headers;
    }

    private static WebResourceResponse emptyResponse(int status, String reason) {
        byte[] json = "{\"error\":\"Estação indisponível\"}".getBytes(StandardCharsets.UTF_8);
        return new WebResourceResponse("application/json", "UTF-8", status, reason,
            responseHeaders(), new ByteArrayInputStream(json));
    }

    /** Insets keep touch targets clear of camera cutouts, navigation gestures and the keyboard. */
    private void configureWindowInsets() {
        if (Build.VERSION.SDK_INT >= 30) {
            getWindow().setDecorFitsSystemWindows(false);
            root.setOnApplyWindowInsetsListener((view, insets) -> {
                Insets safe = insets.getInsets(WindowInsets.Type.systemBars()
                    | WindowInsets.Type.displayCutout() | WindowInsets.Type.ime());
                view.setPadding(safe.left, safe.top, safe.right, safe.bottom);
                return insets;
            });
        } else {
            // Older fullscreen WebViews may ignore ADJUST_RESIZE. Pad only the part still
            // obscured by the IME; when Android already resized the root, this difference is 0.
            root.getViewTreeObserver().addOnGlobalLayoutListener(() -> {
                Rect visible = new Rect();
                root.getWindowVisibleDisplayFrame(visible);
                int[] location = new int[2];
                root.getLocationOnScreen(location);
                int obscured = Math.max(0, location[1] + root.getHeight() - visible.bottom);
                int keyboard = obscured > root.getHeight() / 4 ? obscured : 0;
                if (root.getPaddingBottom() != keyboard) root.setPadding(0, 0, 0, keyboard);
            });
        }
        enterFullscreen();
    }

    @SuppressWarnings("deprecation")
    private void enterFullscreen() {
        if (Build.VERSION.SDK_INT >= 30) {
            WindowInsetsController controller = getWindow().getInsetsController();
            if (controller != null) {
                controller.setSystemBarsBehavior(
                    WindowInsetsController.BEHAVIOR_SHOW_TRANSIENT_BARS_BY_SWIPE);
                controller.hide(WindowInsets.Type.systemBars());
            }
        } else {
            // Keep resize semantics for the keyboard on Android 8–10, without layout-behind flags.
            getWindow().getDecorView().setSystemUiVisibility(View.SYSTEM_UI_FLAG_FULLSCREEN
                | View.SYSTEM_UI_FLAG_HIDE_NAVIGATION | View.SYSTEM_UI_FLAG_IMMERSIVE_STICKY);
        }
        root.requestApplyInsets();
    }

    @Override public void onConfigurationChanged(Configuration newConfig) {
        super.onConfigurationChanged(newConfig);
        // Rotation/folding resizes the existing page; token and current tab remain only in memory.
        enterFullscreen();
    }

    /** Guarda apenas a origem; token é digitado na página e fica somente na memória dela. */
    private void chooseStation() {
        EditText address = new EditText(this);
        address.setInputType(InputType.TYPE_CLASS_TEXT | InputType.TYPE_TEXT_VARIATION_URI);
        address.setSingleLine(true);
        address.setHint("http://192.168.1.20:8765");
        address.setText(gateway);
        AlertDialog dialog = new AlertDialog.Builder(this)
            .setTitle("Estação do drone")
            .setMessage("Endereço do gateway no computador/companion. O celular deve alcançar essa rede.")
            .setView(address).setNegativeButton("Voltar", null)
            .setNeutralButton("Sem estação", (dialogView, which) -> {
                gateway = "";
                getPreferences(MODE_PRIVATE).edit().remove("gateway").apply();
                openStation();
            })
            .setPositiveButton("Abrir", null).create();
        dialog.setOnShowListener(ignored -> dialog.getButton(AlertDialog.BUTTON_POSITIVE)
            .setOnClickListener(view -> {
                try {
                    gateway = normalizeOrigin(address.getText().toString().trim());
                    getPreferences(MODE_PRIVATE).edit().putString("gateway", gateway).apply();
                    openStation();
                    dialog.dismiss();
                } catch (IllegalArgumentException error) {
                    address.setError("Use http(s)://host:porta, sem caminho, token ou credenciais.");
                }
            }));
        dialog.show();
    }

    static String normalizeOrigin(String value) {
        try {
            URI uri = new URI(value);
            String scheme = uri.getScheme() == null ? "" : uri.getScheme().toLowerCase(Locale.ROOT);
            if ((!scheme.equals("http") && !scheme.equals("https")) || uri.getHost() == null
                    || uri.getUserInfo() != null || uri.getQuery() != null || uri.getFragment() != null
                    || (uri.getRawPath() != null && !uri.getRawPath().isEmpty()
                        && !uri.getRawPath().equals("/"))
                    || uri.getPort() == 0 || uri.getPort() > 65535) throw new IllegalArgumentException();
            int port = uri.getPort();
            if ((scheme.equals("http") && port == 80) || (scheme.equals("https") && port == 443)) port = -1;
            return new URI(scheme, null, uri.getHost().toLowerCase(Locale.ROOT), port,
                           null, null, null).toString();
        } catch (Exception error) { throw new IllegalArgumentException(error); }
    }

    private static boolean sameOrigin(String value, String expected) {
        try {
            URI uri = new URI(value);
            String origin = new URI(uri.getScheme(), uri.getUserInfo(), uri.getHost(), uri.getPort(),
                                    null, null, null).toString();
            return normalizeOrigin(origin).equals(expected);
        } catch (Exception error) { return false; }
    }

    private void openStation() {
        pageGeneration++;
        clearSpeech();
        web.stopLoading();
        web.clearHistory();
        web.clearCache(true);
        web.loadUrl(currentOrigin() + "/?app=android");
    }

    /** O serviço de voz do aparelho pode usar a nuvem; só retorna texto para revisão. */
    @SuppressWarnings("deprecation") // Activity é framework puro, sem dependência AndroidX.
    private void startSpeech() {
        if (!resumed || !sameOrigin(web.getUrl(), currentOrigin())) {
            Toast.makeText(this, "Abra a estação antes de falar.", Toast.LENGTH_LONG).show();
            return;
        }
        new AlertDialog.Builder(this)
            .setTitle("Transcrever fala")
            .setMessage("O serviço de voz do aparelho pode enviar áudio à nuvem. "
                + "O texto será mostrado para revisão, sem executar comandos.")
            .setNegativeButton("Cancelar", null)
            .setPositiveButton("Falar", (dialog, which) -> launchSpeech())
            .show();
    }

    @SuppressWarnings("deprecation")
    private void launchSpeech() {
        if (!resumed || !sameOrigin(web.getUrl(), currentOrigin())) return;
        speechOrigin = currentOrigin();
        speechGeneration = pageGeneration;
        speechStarted = SystemClock.elapsedRealtime();
        pendingTranscript = null;
        Intent intent = new Intent(RecognizerIntent.ACTION_RECOGNIZE_SPEECH);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE_MODEL, RecognizerIntent.LANGUAGE_MODEL_FREE_FORM);
        intent.putExtra(RecognizerIntent.EXTRA_LANGUAGE, "pt-BR");
        intent.putExtra(RecognizerIntent.EXTRA_MAX_RESULTS, 1);
        intent.putExtra(RecognizerIntent.EXTRA_PROMPT, "Diga seu pedido; revise o texto ao voltar.");
        try {
            startActivityForResult(intent, SPEECH_REQUEST);
        } catch (ActivityNotFoundException | SecurityException error) {
            clearSpeech();
            Toast.makeText(this, "Reconhecimento de voz indisponível. Digite o pedido na página.",
                Toast.LENGTH_LONG).show();
        }
    }

    @Override @SuppressWarnings("deprecation")
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        super.onActivityResult(requestCode, resultCode, data);
        if (requestCode != SPEECH_REQUEST) return;
        if (resultCode != RESULT_OK || data == null || !validSpeechTarget()) {
            clearSpeech();
            return;
        }
        ArrayList<String> results = data.getStringArrayListExtra(RecognizerIntent.EXTRA_RESULTS);
        if (results == null || results.isEmpty() || results.get(0) == null) {
            clearSpeech();
            return;
        }
        pendingTranscript = results.get(0).trim();
        if (pendingTranscript.isEmpty() || pendingTranscript.length() > 4000) clearSpeech();
        deliverTranscript();
    }

    private boolean validSpeechTarget() {
        return speechOrigin != null && speechOrigin.equals(currentOrigin())
            && speechGeneration == pageGeneration && sameOrigin(web.getUrl(), speechOrigin)
            && SystemClock.elapsedRealtime() - speechStarted < 120_000;
    }

    private void deliverTranscript() {
        // O resultado pode chegar antes de onResume; espere a página estar visível novamente.
        if (!resumed || !hasWindowFocus() || pendingTranscript == null) return;
        if (validSpeechTarget()) {
            String transcript = JSONObject.quote(pendingTranscript);
            web.evaluateJavascript("window.dispatchEvent(new CustomEvent('copilot:transcript',"
                + "{detail:" + transcript + "}))", null);
            Toast.makeText(this, "Texto transcrito. Revise o pedido na página.", Toast.LENGTH_LONG).show();
        }
        clearSpeech();
    }

    private void clearSpeech() {
        speechOrigin = null;
        pendingTranscript = null;
    }

    @Override public void onWindowFocusChanged(boolean hasFocus) {
        super.onWindowFocusChanged(hasFocus);
        if (hasFocus) {
            enterFullscreen();
            deliverTranscript();
        }
    }

    @Override protected void onPause() {
        resumed = false;
        // Transcrição já recebida não pode sobreviver a uma nova suspensão.
        if (pendingTranscript != null) clearSpeech();
        web.evaluateJavascript("window.dispatchEvent(new Event('app:pause'))", null);
        web.onPause();
        super.onPause();
    }

    @Override protected void onResume() {
        super.onResume();
        resumed = true;
        if (web != null) {
            web.onResume();
            web.evaluateJavascript("window.dispatchEvent(new Event('app:resume'))", null);
            deliverTranscript();
        }
    }

    @Override protected void onDestroy() {
        clearSpeech();
        web.stopLoading();
        web.destroy();
        super.onDestroy();
    }
}
