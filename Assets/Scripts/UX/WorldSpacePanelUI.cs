using UnityEngine;
using UnityEngine.UIElements;

namespace Google.XR.ARCoreExtensions.Samples.Geospatial
{
    [RequireComponent(typeof(UIDocument))]
    public class WorldSpacePanelUI : MonoBehaviour
    {
        #region Private Fields
        private UIDocument _uiDocument;
        private Label _popupTitle;
        private Label _popupDescription;
        private Label _popupDistance;
        private Button _dismissButton;
        #endregion

        #region Events & Delegates
        public delegate void DismissClickedAction();
        public event DismissClickedAction OnDismissClicked;
        #endregion

        #region Unity Lifecycle
        private void Awake()
        {
            _uiDocument = GetComponent<UIDocument>();
        }

        private void OnEnable()
        {
            if (_uiDocument == null) return;

            var root = _uiDocument.rootVisualElement;
            if (root != null)
            {
                _popupTitle = root.Q<Label>("PopupTitle");
                _popupDescription = root.Q<Label>("PopupDescription");
                _popupDistance = root.Q<Label>("PopupDistance");
                _dismissButton = root.Q<Button>("DismissButton");

                if (_dismissButton != null)
                {
                    _dismissButton.clicked += OnDismissButtonClicked;
                }
            }
        }

        private void OnDisable()
        {
            if (_dismissButton != null)
            {
                _dismissButton.clicked -= OnDismissButtonClicked;
            }
        }
        #endregion

        #region Public Setters
        public void SetTitle(string text)
        {
            if (_popupTitle != null)
            {
                _popupTitle.text = text;
            }
        }

        public void SetDescription(string text)
        {
            if (_popupDescription != null)
            {
                _popupDescription.text = text;
            }
        }

        public void SetDistance(string text)
        {
            if (_popupDistance != null)
            {
                _popupDistance.text = text;
            }
        }
        #endregion

        #region UI Event Handlers
        private void OnDismissButtonClicked()
        {
            Debug.Log("[WorldSpacePanelUI] Dismiss button clicked.");
            OnDismissClicked?.Invoke();
            
            // By default, let's deactivate the panel when dismissed
            gameObject.SetActive(false);
        }
        #endregion
    }
}
