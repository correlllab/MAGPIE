import { basicSetup, EditorView } from "codemirror"
import { python } from "@codemirror/lang-python"

// Hide the existing textarea
let textarea = document.querySelector('textarea');
textarea.style.display = "none";

// Create a CodeMirror editor with the textarea's contents
let view = new EditorView({
    doc: textarea.value,
    extensions: [
        basicSetup,
        python({}),
    ],
});
// Insert the editor into the document
textarea.insertAdjacentElement("afterend", view.dom);

// When submitting the form, update the textarea with the editor's
// contents so that they're included with the form submission.
textarea.parentElement.onsubmit = function () {
    textarea.value = view.state.doc;
}
